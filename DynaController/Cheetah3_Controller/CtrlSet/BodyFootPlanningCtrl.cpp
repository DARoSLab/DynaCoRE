#include "BodyFootPlanningCtrl.hpp"
#include <Configuration.h>
#include <Cheetah3_Controller/Cheetah3_StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>

#include <Cheetah3_Controller/TaskSet/LinkPosTask.hpp>
#include <Cheetah3_Controller/TaskSet/LinkHeightTask.hpp>
#include <Cheetah3_Controller/TaskSet/LinkOriTask.hpp>
#include <Cheetah3_Controller/TaskSet/JPosTask.hpp>
#include <Cheetah3_Controller/ContactSet/SingleContact.hpp>

BodyFootPlanningCtrl::BodyFootPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    push_down_height_(0.),
    des_jpos_(cheetah3::num_act_joint),
    des_jvel_(cheetah3::num_act_joint),
    des_jacc_(cheetah3::num_act_joint),
    Kp_(cheetah3::num_act_joint),
    Kd_(cheetah3::num_act_joint)
{
    des_jacc_.setZero();
    fr_contact_ = new SingleContact(robot_sys_, cheetah3_link::fr_Foot);
    fl_contact_ = new SingleContact(robot_sys_, cheetah3_link::fl_Foot);
    hr_contact_ = new SingleContact(robot_sys_, cheetah3_link::hr_Foot);
    hl_contact_ = new SingleContact(robot_sys_, cheetah3_link::hl_Foot);
    dim_contact_ = fr_contact_->getDim() + fl_contact_->getDim() + 
        hr_contact_->getDim() + hl_contact_->getDim();

    body_pos_task_ = new LinkPosTask(robot_sys_, cheetah3_link::body);
    body_ori_task_ = new LinkOriTask(robot_sys_, cheetah3_link::body);

    foot_pos_task_ = new LinkPosTask(robot_sys_, swing_foot);
    foot_ori_task_ = new LinkOriTask(robot_sys_, swing_foot);
    total_joint_task_ = new JPosTask();

    std::vector<bool> act_list;
    act_list.resize(cheetah3::num_qdot, true);
    for(int i(0); i<cheetah3::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(cheetah3::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[fr_contact_->getFzIndex()] = 0.01;
    wblc_data_->W_rf_[fr_contact_->getDim() + fl_contact_->getFzIndex()] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(cheetah3::num_act_joint, -500.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(cheetah3::num_act_joint, 500.);

    kin_wbc_contact_list_.clear();
    int rf_idx_offset(0);

    // Create Minimum jerk objects
    for(size_t i = 0; i < 3; i++){
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
    printf("[Configuration BodyFootPlanning Controller] Constructed\n");
}

void BodyFootPlanningCtrl::OneStep(void* _cmd){   
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<cheetah3::num_act_joint; ++i){
        ((Cheetah3_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Cheetah3_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Cheetah3_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyFootPlanningCtrl::_contact_setup(){
    fr_contact_->UpdateContactSpec();
    fl_contact_->UpdateContactSpec();
    contact_list_.push_back(fr_contact_);
    contact_list_.push_back(fl_contact_);
}

void BodyFootPlanningCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    wblc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(cheetah3::num_virtual, cheetah3::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(cheetah3::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    //dynacore::pretty_print(wblc_data_->Fr_, std::cout, "Fr");
    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
}

void BodyFootPlanningCtrl::_task_setup(){
    // Body Pos
    double body_height_cmd;
    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_pos_[2];

    dynacore::Vector vel_des(3); vel_des.setZero();
    dynacore::Vector acc_des(3); acc_des.setZero();
    dynacore::Vect3 des_pos = ini_body_pos_;
    des_pos[2] = body_height_cmd;
    body_pos_task_->UpdateTask(&(des_pos), vel_des, acc_des);
    
    // Body & Foot Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);
    dynacore::Vector ang_vel_des(3); ang_vel_des.setZero();
    dynacore::Vector ang_acc_des(3); ang_acc_des.setZero();

    body_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);
    foot_ori_task_->UpdateTask(&(des_quat), ang_vel_des, ang_acc_des);

    _CheckPlanning();
    // Foot Pos Task Setup 
    _foot_pos_task_setup();

    // Full joint task
    dynacore::Vector jpos_des = sp_->jpos_ini_;
    dynacore::Vector zero(cheetah3::num_act_joint); zero.setZero();
    total_joint_task_->UpdateTask(&(jpos_des), zero, zero);

    // Task Update
    task_list_.push_back(foot_pos_task_);
    task_list_.push_back(body_pos_task_);
    task_list_.push_back(body_ori_task_);
    task_list_.push_back(foot_ori_task_);
    task_list_.push_back(total_joint_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, kin_wbc_contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
}

void BodyFootPlanningCtrl::_foot_pos_task_setup(){
    //_GetSinusoidalSwingTrajectory();
    _GetBsplineSwingTrajectory();

    double traj_time = state_machine_time_ - half_swing_time_;
    if(state_machine_time_ > half_swing_time_){
        double pos, vel, acc;
        for(int i(0); i<3; ++i){
            min_jerk_offset_[i]->getPos(traj_time, pos);  
            min_jerk_offset_[i]->getVel(traj_time, vel);
            min_jerk_offset_[i]->getAcc(traj_time, acc);

            curr_foot_pos_des_[i] += pos;
            curr_foot_vel_des_[i] += vel;
            curr_foot_acc_des_[i] += acc;
        }
    }
    if(state_machine_time_> end_time_){
        for(int i(0); i<foot_pos_task_->getDim(); ++i){
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] = 
            -push_down_height_ - 0.1*(state_machine_time_ - end_time_);
    }

    dynacore::Vector foot_vel_des(foot_pos_task_->getDim()); foot_vel_des.setZero();
    dynacore::Vector foot_acc_des(foot_pos_task_->getDim()); foot_acc_des.setZero();
    foot_vel_des = curr_foot_vel_des_;
    foot_acc_des = curr_foot_acc_des_;

    foot_pos_task_->UpdateTask(
            &(curr_foot_pos_des_), 
            foot_vel_des, 
            foot_acc_des);
}

void BodyFootPlanningCtrl::_CheckPlanning(){
    if( (state_machine_time_ > 0.5 * end_time_) && b_replanning_ && !b_replaned_) {
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);

        dynacore::Vect3 target_offset;
        // X, Y target is originally set by intial_traget_loc
        for(int i(0); i<2; ++i)
            target_offset[i] = target_loc[i] - initial_target_loc_[i];

        // Foot height (z) is set by the initial height
        target_offset[2] = 0.; //target_loc[2] - ini_foot_pos_[2];

        _SetMinJerkOffset(target_offset);
        b_replaned_ = true;
    }
}

void BodyFootPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){}

void BodyFootPlanningCtrl::FirstVisit(){
    b_replaned_ = false;
    ini_config_ = sp_->Q_;
    robot_sys_->getPos(cheetah3_link::body, ini_body_pos_);
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    initial_target_loc_[0] = sp_->Q_[0];
    initial_target_loc_[1] = sp_->Q_[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_;

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    dynacore::Vect3 foot_pos_offset; foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;
    _SetMinJerkOffset(foot_pos_offset);

    dynacore::Vect3 com_vel;
    robot_sys_->getCoMPosition(ini_com_pos_);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = ini_com_pos_[0];   
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

}

void BodyFootPlanningCtrl::_SetMinJerkOffset(const dynacore::Vect3 & offset){
    // Initialize Minimum Jerk Parameter Containers
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero(); 
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i]; 

        min_jerk_offset_[i]->setParams(
                init_params, final_params,
                0., half_swing_time_);  
    }
}

bool BodyFootPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
                state_machine_time_, end_time_);
        return true;
    }
    return false;
}

void BodyFootPlanningCtrl::CtrlInitialization(
        const std::string & setting_file_name){
    ini_base_height_ = sp_->Q_[cheetah3_joint::virtual_Z];
    std::vector<double> tmp_vec;

    // Setting Parameters
    ParamHandler handler(Cheetah3ConfigPath + setting_file_name + ".yaml");
    handler.getValue("swing_height", swing_height_);
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("default_target_foot_location", tmp_vec);
    for(int i(0); i<3; ++i){
        default_target_loc_[i] = tmp_vec[i];
    }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<cheetah3::num_act_joint; ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<cheetah3::num_act_joint; ++i){
        Kd_[i] = tmp_vec[i];
    }

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }

    handler.getVector("foot_landing_offset", foot_landing_offset_);

    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
    //printf("[Body Foot JPos Planning Ctrl] Parameter Setup Completed\n");
}

BodyFootPlanningCtrl::~BodyFootPlanningCtrl(){
    delete wblc_;
    delete fl_contact_;
    delete fr_contact_;
}

void BodyFootPlanningCtrl::_GetBsplineSwingTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(state_machine_time_, pos);
    foot_traj_.getCurveDerPoint(state_machine_time_, 1, vel);
    foot_traj_.getCurveDerPoint(state_machine_time_, 2, acc);

    for(int i(0); i<3; ++i){
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
}
void BodyFootPlanningCtrl::_GetSinusoidalSwingTrajectory(){
    curr_foot_acc_des_.setZero();
    for (int i(0); i<2; ++i){
        curr_foot_pos_des_[i] = 
            dynacore::smooth_changing(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
        curr_foot_vel_des_[i] = 
            dynacore::smooth_changing_vel(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
        curr_foot_acc_des_[i] = 
            dynacore::smooth_changing_acc(ini_foot_pos_[i], initial_target_loc_[i], 
                    end_time_, state_machine_time_);
    }
    // for Z (height)
    double amp(swing_height_/2.);
    double omega ( 2.*M_PI /end_time_ );

    curr_foot_pos_des_[2] = 
        ini_foot_pos_[2] + amp * (1-cos(omega * state_machine_time_));
    curr_foot_vel_des_[2] = 
        amp * omega * sin(omega * state_machine_time_);
    curr_foot_acc_des_[2] = 
        amp * omega * omega * cos(omega * state_machine_time_);
}

void BodyFootPlanningCtrl::_SetBspline(
        const dynacore::Vect3 & st_pos, 
        const dynacore::Vect3 & des_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    dynacore::Vect3 middle_pos;

    middle_pos = (st_pos + des_pos)/2.;
    middle_pos[2] = swing_height_;

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = 0.;
        init[i+6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = -0.5;
    fin[8] = 5.;
    foot_traj_.SetParam(init, fin, middle_pt, end_time_);

    delete [] *middle_pt;
    delete [] middle_pt;    
}


