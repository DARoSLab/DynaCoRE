#include "CoMFootJPosPlanningCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/CoMFootJPosTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#define MEASURE_TIME_WBDC 0

#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

CoMFootJPosPlanningCtrl::CoMFootJPosPlanningCtrl(
        RobotSystem* robot, int swing_foot, Planner* planner):
    Controller(robot),
    swing_foot_(swing_foot),
    num_planning_(0),
    planning_frequency_(0.),
    replan_moment_(0.),
    push_down_height_(0.),
    ctrl_start_time_(0.),
    b_contact_switch_check_(false)
{
    planner_ = planner;
    curr_jpos_des_.setZero();
    curr_jvel_des_.setZero();
    curr_jacc_des_.setZero();

    com_foot_task_ = new CoMFootJPosTask(robot, swing_foot);
    if(swing_foot == mercury_link::leftFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::rightFoot); 
        swing_leg_jidx_ = mercury_joint::leftAbduction;
    }
    else if(swing_foot == mercury_link::rightFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::leftFoot);
        swing_leg_jidx_ = mercury_joint::rightAbduction;
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;


    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(
                com_foot_task_->getDim() + 
                single_contact_->getDim(), 1000.0);

    wbdc_rotor_data_->cost_weight[0] = 0.0001; // X
    wbdc_rotor_data_->cost_weight[1] = 0.0001; // Y
    wbdc_rotor_data_->cost_weight[5] = 0.0001; // Yaw

    wbdc_rotor_data_->cost_weight.tail(single_contact_->getDim()) = 
        dynacore::Vector::Constant(single_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[com_foot_task_->getDim() + 2]  = 0.001; // Fr_z


    com_estimator_ = new LIPM_KalmanFilter();
    sp_ = Mercury_StateProvider::getStateProvider();
    // printf("[CoMFoot Controller] Constructed\n");
}

CoMFootJPosPlanningCtrl::~CoMFootJPosPlanningCtrl(){
    delete com_foot_task_;
    delete single_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void CoMFootJPosPlanningCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _single_contact_setup();
    _task_setup();
    _com_foot_ctrl(gamma);

    _PostProcessing_Command();
}
void CoMFootJPosPlanningCtrl::_com_foot_ctrl(dynacore::Vector & gamma){
#if MEASURE_TIME_WBDC 
    static int time_count(0);
    time_count++;
    std::chrono::high_resolution_clock::time_point t1 
        = std::chrono::high_resolution_clock::now();
#endif
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma.head(mercury::num_act_joint) = fb_cmd;
    gamma.tail(mercury::num_act_joint) = wbdc_rotor_data_->cmd_ff;

#if MEASURE_TIME_WBDC 
    std::chrono::high_resolution_clock::time_point t2 
        = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span1 
        = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    if(time_count%500 == 1){
        std::cout << "[com foot planning] WBDC_Rotor took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
    }
#endif

    int offset(0);
    if(swing_foot_ == mercury_link::rightFoot) offset = 3;
    dynacore::Vector reaction_force = 
        (wbdc_rotor_data_->opt_result_).tail(single_contact_->getDim());
    for(int i(0); i<3; ++i)
        sp_->reaction_forces_[i + offset] = reaction_force[i];

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
}


void CoMFootJPosPlanningCtrl::_task_setup(){
    dynacore::Vector pos_des(3 + 4 + 3);
    dynacore::Vector vel_des(com_foot_task_->getDim());
    dynacore::Vector acc_des(com_foot_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // CoM Pos
    pos_des.head(3) = ini_com_pos_;
    if(b_set_height_target_) pos_des[2] = des_com_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();

    _CoMEstiamtorUpdate();
    _CheckPlanning();

    double traj_time = state_machine_time_ - replan_moment_;
    // Swing Foot Config Task
    double pos[3];
    double vel[3];
    double acc[3];

    // printf("time: %f\n", state_machine_time_);
    foot_traj_.getCurvePoint(traj_time, pos);
    foot_traj_.getCurveDerPoint(traj_time, 1, vel);
    foot_traj_.getCurveDerPoint(traj_time, 2, acc);
    // printf("pos:%f, %f, %f\n", pos[0], pos[1], pos[2]);

    for(int i(0); i<3; ++i){
        sp_->jpos_des_[swing_leg_jidx_ - mercury::num_virtual + i] = pos[i];
        sp_->jvel_des_[swing_leg_jidx_ - mercury::num_virtual + i] = vel[i];

        curr_jpos_des_[i] = pos[i];
        curr_jvel_des_[i] = vel[i];
        curr_jacc_des_[i] = acc[i];
 
        pos_des[i + 7] = curr_jpos_des_[i];
        vel_des[i + 6] = curr_jvel_des_[i];
        acc_des[i + 6] = curr_jacc_des_[i];
    }
    // dynacore::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
    // Push back to task list
    com_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(com_foot_task_);
}

void CoMFootJPosPlanningCtrl::_CheckPlanning(){
    if( state_machine_time_ > 
            (end_time_/(planning_frequency_ + 1.) * (num_planning_ + 1.) + 0.002) ){

        //if(state_machine_time_ > 0.5 * end_time_ + 0.002 && (num_planning_ < 1)){
        //+ 0.002 is to account one or two more ticks before the end of phase
        _Replanning();
        ++num_planning_;
    }

    // Earlier planning
    //_Replanning();
    //++num_planning_;
    //}

    //printf("time (state/end): %f, %f\n", state_machine_time_, end_time_);
    // printf("planning freq: %f\n", planning_frequency_);
    // printf("num_planning: %i\n", num_planning_);
}

void CoMFootJPosPlanningCtrl::_Replanning(){
    dynacore::Vect3 com_pos, com_vel, target_loc;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);

    // Estimated value used
    for(int i(0); i<2; ++i){
        com_pos[i] = sp_->estimated_com_state_[i];
        com_vel[i] = sp_->estimated_com_state_[i + 2];
    }
    // x vel is zero
    com_vel[0] = 0.;
    // Using Y vel from imu
    com_vel[1] = sp_->com_state_imu_[3];

    OutputReversalPL pl_output;
    ParamReversalPL pl_param;
    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_;

    pl_param.des_loc = sp_->des_location_;
    pl_param.stance_foot_loc = sp_->global_pos_local_;

    if(swing_foot_ == mercury_link::leftFoot)
        pl_param.b_positive_sidestep = true;
    else
        pl_param.b_positive_sidestep = false;

    planner_->getNextFootLocation(com_pos + sp_->global_pos_local_,
            com_vel,
            target_loc,
            &pl_param, &pl_output);
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;

    // dynacore::pretty_print(target_loc, std::cout, "planed foot loc");
    // dynacore::pretty_print(sp_->global_pos_local_, std::cout, "global loc");

    target_loc -= sp_->global_pos_local_;
    //target_loc[2] -= push_down_height_;
    target_loc[2] = default_target_loc_[2];
    //dynacore::pretty_print(target_loc, std::cout, "next foot loc");
    //curr_foot_acc_des_.setZero();
    //curr_foot_vel_des_.setZero();
    _SetBspline(curr_jpos_des_, curr_jvel_des_, curr_jacc_des_, target_loc);
}

void CoMFootJPosPlanningCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void CoMFootJPosPlanningCtrl::FirstVisit(){
   // printf("[CoM Foot Ctrl] Start\n");
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    ini_swing_leg_config_ = sp_->Q_.segment(swing_leg_jidx_, 3);

    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    dynacore::Vect3 zero;
    zero.setZero();
    dynacore::Vect3 target_loc = default_target_loc_;
    target_loc[0] = sp_->Q_[0];
    target_loc[1] += sp_->Q_[1];
    target_loc[2] = ini_foot_pos_[2];
    //default_target_loc_[2] = 0.0;
    default_target_loc_[2] -= push_down_height_;
    _SetBspline(ini_swing_leg_config_, zero, zero, target_loc);

    // _Replanning();
    num_planning_ = 0;

    dynacore::Vect3 com_pos, com_vel;
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = com_pos[0];   input_state[1] = com_pos[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

    com_estimator_->EstimatorInitialization(input_state);
    _CoMEstiamtorUpdate();

     //dynacore::pretty_print(ini_foot_pos_, std::cout, "ini foot pos");
     //dynacore::pretty_print(target_loc, std::cout, "target loc");
}

void CoMFootJPosPlanningCtrl::_CoMEstiamtorUpdate(){
    dynacore::Vect3 com_pos, com_vel;
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = com_pos[0];   input_state[1] = com_pos[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];
    com_estimator_->InputData(input_state);
    com_estimator_->Output(sp_->estimated_com_state_);
}

void CoMFootJPosPlanningCtrl::_SetBspline(
        const dynacore::Vect3 & st_config,
        const dynacore::Vect3 & st_jvel,
        const dynacore::Vect3 & st_jacc,
        const dynacore::Vect3 & target_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];

    // printf("time (state/end): %f, %f\n", state_machine_time_, end_time_);
    double portion = (1./end_time_) * (end_time_/2. - state_machine_time_);

    // Find Target Config
    dynacore::Vector config_sol;
    inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, target_pos, 
            sp_->Q_, config_sol);
    dynacore::Vect3 target_config = config_sol.segment(swing_leg_jidx_, 3);

    // Find Mid Config
    dynacore::Vect3 st_pos;
    inv_kin_.getFootPosAtVerticalPosture(swing_foot_, st_config, sp_->Q_, st_pos);
    dynacore::Vect3 middle_pos;
    if(portion > 0.){
        middle_pos = (st_pos + target_pos)*portion;
        middle_pos[2] = swing_height_;
    } else {
        middle_pos = (st_pos + target_pos)/2.;
    }
    inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, middle_pos, 
            sp_->Q_, config_sol);
    dynacore::Vect3 mid_config = config_sol.segment(swing_leg_jidx_, 3);
    
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_config[i];
        init[i+3] = st_jvel[i];
        init[i+6] = st_jacc[i];
        // Final
        fin[i] = target_config[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // mid
        middle_pt[0][i] = mid_config[i];
   }

    foot_traj_.SetParam(init, fin, middle_pt, end_time_ - replan_moment_);

    delete [] *middle_pt;
    delete [] middle_pt;
}


void CoMFootJPosPlanningCtrl::LastVisit(){
}

bool CoMFootJPosPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        //printf("[CoM Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
        //state_machine_time_, end_time_);
        return true;
    }
    // Swing foot contact = END
    if(b_contact_switch_check_){
        bool contact_happen(false);
        if(swing_foot_ == mercury_link::leftFoot && sp_->b_lfoot_contact_){
            contact_happen = true;
        }
        if(swing_foot_ == mercury_link::rightFoot && sp_->b_rfoot_contact_){
            contact_happen = true;
        }
        if(state_machine_time_ > end_time_ * 0.5 && contact_happen){
            printf("[CoM Foot JPos Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void CoMFootJPosPlanningCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_com_pos_);
    std::vector<double> tmp_vec;

    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("swing_height", swing_height_);
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("default_target_foot_location", tmp_vec);
    for(int i(0); i<3; ++i){
        default_target_loc_[i] = tmp_vec[i];
    }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMFootJPosTask*)com_foot_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMFootJPosTask*)com_foot_task_)->Kd_vec_[i] = tmp_vec[i];
    }

    static bool b_compute_eigenvalue(true);
    if(b_compute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_compute_eigenvalue = false;
    }
    //printf("[CoM Foot JPos Planning Ctrl] Parameter Setup Completed\n");
}
