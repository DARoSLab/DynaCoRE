#include "BodyJPosSwingPlanningCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#define MEASURE_TIME_WBDC 0

#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

BodyJPosSwingPlanningCtrl::BodyJPosSwingPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    push_down_height_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint)
{
    curr_jpos_des_.setZero();
    curr_jvel_des_.setZero();
    curr_jacc_des_.setZero();


    prev_ekf_vel.setZero();
    acc_err_ekf.setZero();


    config_body_foot_task_ = new ConfigTask();
    if(swing_foot == mercury_link::leftFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::rightFoot); 
        swing_leg_jidx_ = mercury_joint::leftAbduction;
    }
    else if(swing_foot == mercury_link::rightFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::leftFoot);
        swing_leg_jidx_ = mercury_joint::rightAbduction;
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    task_kp_ = dynacore::Vector::Zero(config_body_foot_task_->getDim());
    task_kd_ = dynacore::Vector::Zero(config_body_foot_task_->getDim());

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;


    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);

    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(
                config_body_foot_task_->getDim() + 
                single_contact_->getDim(), 100.0);

    // for(int i(0); i<mercury::num_virtual; ++i) 
    //     wbdc_rotor_data_->cost_weight[i] = 350.;

    wbdc_rotor_data_->cost_weight.tail(single_contact_->getDim()) = 
        dynacore::Vector::Constant(single_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[config_body_foot_task_->getDim() + 2]  = 0.001; // Fr_z

    // Create Minimum jerk objects
    for(size_t i = 0; i < 3; i++){
        min_jerk_cartesian.push_back(new MinJerk_OneDimension());
    }

    printf("[BodyFootJPosPlanning Controller] Constructed\n");
}

BodyJPosSwingPlanningCtrl::~BodyJPosSwingPlanningCtrl(){
    delete config_body_foot_task_;
    delete single_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void BodyJPosSwingPlanningCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _single_contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}
void BodyJPosSwingPlanningCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    int offset(0);
    if(swing_foot_ == mercury_link::rightFoot) offset = 3;
    dynacore::Vector reaction_force = 
        (wbdc_rotor_data_->opt_result_).tail(single_contact_->getDim());
    for(int i(0); i<3; ++i)
        sp_->reaction_forces_[i + offset] = reaction_force[i];

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
}


void BodyJPosSwingPlanningCtrl::_task_setup(){
    dynacore::Vector pos_des(config_body_foot_task_->getDim());
    dynacore::Vector vel_des(config_body_foot_task_->getDim());
    dynacore::Vector acc_des(config_body_foot_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();

    dynacore::convert(rpy_des, des_quat);

    _CoMEstiamtorUpdate();
    _CheckPlanning();        

    double traj_time = state_machine_time_ - replan_moment_;
    // Swing Foot Config Task
    double pos[3];
    double vel[3];
    double acc[3];

    foot_traj_.getCurvePoint(traj_time, pos);
    foot_traj_.getCurveDerPoint(traj_time, 1, vel);
    foot_traj_.getCurveDerPoint(traj_time, 2, acc);

    for(int i(0); i<3; ++i){
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }

    dynacore::Vector config_sol, qdot_cmd, qddot_cmd;
    inv_kin_.getSingleSupportFullConfigSeperation(
            sp_->Q_, des_quat, target_height, 
            swing_foot_, curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_,
            config_sol, qdot_cmd, qddot_cmd);

    // Get Minimum Jerk Trajectory
    for(size_t i = 0; i < 3; i++){
        min_jerk_cartesian[i]->getPos(traj_time, pos[i]);
        min_jerk_cartesian[i]->getVel(traj_time, vel[i]);
        min_jerk_cartesian[i]->getAcc(traj_time, acc[i]);        

        sp_->test_minjerk_pos[i] = pos[i];
        sp_->test_minjerk_vel[i] = vel[i];
        sp_->test_minjerk_acc[i] = acc[i];
    }

    //TEST
    if(num_planning_ < 1){
    //if(num_planning_ < 2){
        swing_leg_traj_.getCurvePoint(traj_time, pos);
        swing_leg_traj_.getCurveDerPoint(traj_time, 1, vel);
        swing_leg_traj_.getCurveDerPoint(traj_time, 2, acc);
    }


    for(int i(0); i<3; ++i){
        config_sol[swing_leg_jidx_ + i] = pos[i];
        qdot_cmd[swing_leg_jidx_ + i] = vel[i];
        qddot_cmd[swing_leg_jidx_ + i] = acc[i];

        curr_jpos_des_[i] = pos[i];
        curr_jvel_des_[i] = vel[i];
        curr_jacc_des_[i] = acc[i];
    }

    for (int i(0); i<mercury::num_act_joint; ++i){
        pos_des[mercury::num_virtual + i] = config_sol[mercury::num_virtual + i];  
        vel_des[mercury::num_virtual + i] = qdot_cmd[mercury::num_virtual + i];
        acc_des[mercury::num_virtual + i] = qddot_cmd[mercury::num_virtual + i];

        des_jpos_[i] = pos_des[mercury::num_virtual + i];
        des_jvel_[i] = vel_des[mercury::num_virtual + i];
    }
    // For save
     for(int i(0); i<mercury::num_act_joint; ++i){
        sp_->jacc_des_[i] = qddot_cmd[i + mercury::num_virtual];
    }

   //dynacore::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
    // Push back to task list
    config_body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(config_body_foot_task_);
}

void BodyJPosSwingPlanningCtrl::_SetCartesianMinJerk(
        const dynacore::Vect3 & st_pos,
        const dynacore::Vect3 & st_vel,
        const dynacore::Vect3 & st_acc,
        const dynacore::Vect3 & target_pos){

    // Initialize Minimum Jerk Parameter Containers
    std::vector< dynacore::Vect3 > min_jerk_cartesian_initial_params;
    std::vector< dynacore::Vect3 > min_jerk_cartesian_final_params;   
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Initialize Time parameters
    double t_start_min_jerk = 0.0;
    double t_final_min_jerk = (end_time_ - replan_moment_);

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        init_params.setZero(); final_params.setZero();
        // Set Dimension i's initial pos, vel and acceleration
        init_params[0] = st_pos[i]; init_params[1] = st_vel[i];  init_params[2] = st_acc[i];
        // Set Dimension i's final pos, vel, acceleration
        final_params[0] = target_pos[i]; final_params[1] = 0.0;  final_params[2] = 0.0;
        // Add to the parameter vector 
        min_jerk_cartesian_initial_params.push_back(init_params);
        min_jerk_cartesian_final_params.push_back(final_params);
    }

    // Set Minimum Jerk Parameters for each dimension
    for(size_t i = 0; i < 3; i++){
        min_jerk_cartesian[i]->setParams(
                min_jerk_cartesian_initial_params[i], 
                min_jerk_cartesian_final_params[i],
                t_start_min_jerk, t_final_min_jerk);  
        // min_jerk_cartesian[i]->printParameters();  
    }
}


void BodyJPosSwingPlanningCtrl::_CheckPlanning(){
    if( state_machine_time_ > 
            (end_time_/(planning_frequency_ + 1.) * (num_planning_ + 1.) + 0.002) ){
        //if(state_machine_time_ > 0.5 * end_time_ + 0.002 && (num_planning_ < 1)){
        //+ 0.002 is to account one or two more ticks before the end of phase
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);
        //_getHurstPlan(target_loc);
        dynacore::Vector guess_q = sp_->Q_;
        // dynacore::Vector guess_q = ini_config_;
        _SetBspline(guess_q, 
                curr_jpos_des_, curr_jvel_des_, curr_jacc_des_, target_loc);
        _SetBspline(
                curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_, target_loc);

        dynacore::Vector config_sol;
        dynacore::Vect3 target_config;
        inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, target_loc, 
                guess_q, config_sol);
        for(int i(0); i<3; ++i){ 
            target_config[i] = config_sol[swing_leg_jidx_ + i];
        }
        _SetCartesianMinJerk(curr_jpos_des_, curr_jvel_des_, curr_jacc_des_, target_config);
        ++num_planning_;
    }


    // To use the Hurst planner, use this block of code and ensure that the initial_planning parameter is false
    // if ((state_machine_time_ > (end_time_/2.0)) && num_planning_ == 0){
    //     dynacore::Vect3 target_loc;
    //     _getHurstPlan(target_loc);
    //     dynacore::Vector guess_q = sp_->Q_;
    //     _SetBspline(guess_q, 
    //         curr_jpos_des_, curr_jvel_des_, curr_jacc_des_, target_loc);
    //     _SetBspline(
    //         curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_, target_loc);
    //     ++num_planning_;
    // }


    //}
}

void BodyJPosSwingPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
    dynacore::Vect3 del_com_pos;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);

    // Average velocity computation
    for(int i(0); i<2; ++i){ 
        sp_->average_vel_[i] = (sp_->Q_[i] - ini_config_[i])/state_machine_time_;
    }

    // TEST
    for(int i(0); i<2; ++i){
        // com_pos[i] = sp_->Q_[i] + body_pt_offset_[i];
        com_pos[i] += body_pt_offset_[i];
         com_vel[i] = sp_->average_vel_[i]; 
        //com_vel[i] = sp_->ekf_body_vel_[i]; 
    }

    printf("planning com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);


    OutputReversalPL pl_output;
    ParamReversalPL pl_param;

    pl_param.swing_time = end_time_ - state_machine_time_
        + transition_time_ * transition_phase_ratio_
        + stance_time_ * double_stance_ratio_ - swing_time_reduction_;

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

    // TEST
    if(sp_->num_step_copy_ < 2){
        target_loc[0] = sp_->Q_[0] + default_target_loc_[0];
        target_loc[1] = sp_->Q_[1] + default_target_loc_[1];
    }

    //target_loc[2] -= push_down_height_;
    target_loc[2] = default_target_loc_[2];
    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
    //curr_foot_acc_des_.setZero();
    //curr_foot_vel_des_.setZero();
}


void BodyJPosSwingPlanningCtrl::_getHurstPlan(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
    // Direct value used
    robot_sys_->getCoMPosition(com_pos);
    robot_sys_->getCoMVelocity(com_vel);
    // Set Z height of target location
    target_loc[2] = default_target_loc_[2];

    // Get current swing foot position:
    dynacore::Vect3 leftFoot_position; leftFoot_position.setZero();
    dynacore::Vect3 rightFoot_position; rightFoot_position.setZero();    
    robot_sys_->getPos(mercury_link::leftFoot, leftFoot_position);
    robot_sys_->getPos(mercury_link::rightFoot, rightFoot_position);

    dynacore::Vect3 localFoot_startPos; localFoot_startPos.setZero();

    double dx_limit_upper = 0.0;
    double dx_limit_lower = 0.0;     
    double dy_limit_upper = 0.0;
    double dy_limit_lower = 0.0;     

    // Right Swing Phase
    if (sp_->phase_copy_ == 4){
        printf("right swing phase\n");
        localFoot_startPos = rightFoot_position - leftFoot_position;

        dx_limit_upper = 0.35;
        dx_limit_lower = -0.35;  

        // dy_limit_upper = -0.15;
        // dy_limit_lower = -0.50; 
        dy_limit_upper = -0.15;
        dy_limit_lower = -0.55;        

    }else if(sp_-> phase_copy_ == 8){
        printf("left swing phase\n");
        // Left Swing Phase:
        localFoot_startPos = leftFoot_position - rightFoot_position;

        dx_limit_upper = 0.35;
        dx_limit_lower = -0.35;       
        // dy_limit_upper = 0.50;
        // dy_limit_lower = 0.15;
        dy_limit_upper = 0.55;
        dy_limit_lower = 0.15;

        // dy_limit_upper = 0.55;
        // dy_limit_lower = 0.15;


    }
    // dynacore::pretty_print(localFoot_startPos, std::cout, "localFoot_startPos");
    // dynacore::pretty_print(sp_->ekf_body_pos_, std::cout, "ekf_body_pos_");
    // dynacore::pretty_print(sp_->ekf_body_vel_, std::cout, "ekf_body_vel_");        


    // double ki_x = 1e-3;
    // double ki_y = 1e-3;    
    // double kd_x = 0.01;//1e-6;
    // double kd_y = 0.01;//1e-6;

    // double ki_x = 0.01;//0.01;
    // double ki_y = 0.0001;    
    // double kd_x = 0.01;//0.0 //1e-6;
    // double kd_y = 0.001;//0.0;//1e-6;    

    double kp_x = 1.0;
    double kp_y = 1.0;    
    double ki_x = 0.0;//0.01;
    double ki_y = 0.0;    
    double kd_x = 0.0;//0.0 //1e-6;
    double kd_y = 0.0;//0.0;//1e-6;        
    target_loc[0] = localFoot_startPos[0] + kp_x*sp_->ekf_body_vel_[0] + ki_x*(sp_->ekf_body_pos_[0]) + kd_x*(sp_->ekf_body_vel_[0] - prev_ekf_vel[0]);
    target_loc[1] = localFoot_startPos[1] + kp_y*sp_->ekf_body_vel_[1] + ki_y*(sp_->ekf_body_pos_[1]) + kd_y*(sp_->ekf_body_vel_[1] - prev_ekf_vel[1]);

    // With Velocity Tracking
    // double kv_x = 0.00;
    // double kv_y = 0.00;    
    // double des_x_vel = -0.007;
    // target_loc[0] = localFoot_startPos[0] + kp_x_*(sp_->ekf_body_vel_[0] - des_x_vel) + ki_x*(sp_->ekf_body_pos_[0] - des_x_vel*mercury::servo_rate ) + kd_x*(sp_->ekf_body_vel_[0] - prev_ekf_vel[0]) + kv_x*sp_->ekf_body_vel_[0];
    // target_loc[1] = localFoot_startPos[1] + kp_y_*sp_->ekf_body_vel_[1] + ki_y*(sp_->ekf_body_pos_[1]) + kd_y*(sp_->ekf_body_vel_[1] - prev_ekf_vel[1]);

    // Accumulated Error
    // target_loc[0] = localFoot_startPos[0] + kp_x*sp_->ekf_body_vel_[0] + ki_x*(acc_err_ekf[0]) + kd_x*(sp_->ekf_body_vel_[0] - prev_ekf_vel[0]);
    // target_loc[1] = localFoot_startPos[1] + kp_y*sp_->ekf_body_vel_[1] + ki_y*(acc_err_ekf[1]) + kd_y*(sp_->ekf_body_vel_[1] - prev_ekf_vel[1]);
    // acc_err_ekf[0] += sp_->ekf_body_pos_[0];
    // acc_err_ekf[1] += sp_->ekf_body_pos_[1];    

    prev_ekf_vel[0] = sp_->ekf_body_vel_[0];
    prev_ekf_vel[1] = sp_->ekf_body_vel_[1];

    // target_loc[0] = localFoot_startPos[0] + kp_x_*com_vel[0] + ki_x*(com_pos[0]) + kd_x*(com_vel[0] - com_vel[0]);
    // target_loc[1] = localFoot_startPos[1] + kp_y_*com_vel[1] + ki_y*(com_pos[1]) + kd_y*(com_vel[1] - com_vel[1]);
    // prev_ekf_vel[0] = com_vel[0];
    // prev_ekf_vel[1] = com_vel[1];




    if (target_loc[0] >= dx_limit_upper){
        target_loc[0] = dx_limit_upper;
        printf("dx upper limit hit \n");
    }else if (target_loc[0] <= dx_limit_lower){
        target_loc[0] = dx_limit_lower;
        printf("dx lower limit hit \n");
    }    


    if (target_loc[1] >= dy_limit_upper){
        target_loc[1] = dy_limit_upper;
        printf("dy upper limit hit \n");        
    }else if (target_loc[1] <= dy_limit_lower){
        target_loc[1] = dy_limit_lower;
        printf("dy lower limit hit \n");        
    }    


}


void BodyJPosSwingPlanningCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void BodyJPosSwingPlanningCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;

    ini_swing_leg_config_ = sp_->Q_.segment(swing_leg_jidx_, 3);
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    dynacore::Vect3 zero;
    zero.setZero();

    dynacore::Vect3 target_loc = default_target_loc_;
    target_loc[0] += sp_->Q_[0];
    target_loc[1] += sp_->Q_[1];
    target_loc[2] = ini_foot_pos_[2] - push_down_height_;
    // Compute JPos trajectory


    if(b_initial_planning_){
        _Replanning(target_loc);
    }
    _SetBspline(ini_config_, ini_swing_leg_config_, zero, zero, target_loc);
    _SetBspline(ini_foot_pos_, zero, zero, target_loc);
    default_target_loc_[2] = target_loc[2];


    num_planning_ = 0;

    dynacore::Vect3 com_vel;
    robot_sys_->getCoMPosition(ini_com_pos_);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = ini_com_pos_[0];   
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

    com_estimator_->EstimatorInitialization(input_state);
    _CoMEstiamtorUpdate();

    //dynacore::pretty_print(ini_foot_pos_, std::cout, "ini foot pos");
    //dynacore::pretty_print(target_loc, std::cout, "target loc");
}

void BodyJPosSwingPlanningCtrl::_SetBspline(
        const dynacore::Vector & guess_q,
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
            guess_q, config_sol);
    dynacore::Vect3 target_config = config_sol.segment(swing_leg_jidx_, 3);

    // Find Mid Config
    dynacore::Vect3 st_pos;
    inv_kin_.getFootPosAtVerticalPosture(swing_foot_, st_config, guess_q, st_pos);
    dynacore::Vect3 middle_pos;
    if(portion > 0.){
        middle_pos = (st_pos + target_pos)*portion;
        middle_pos[2] = swing_height_ + target_pos[2];
    } else {
        middle_pos = (st_pos + target_pos)/2.;
    }
    inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, middle_pos, 
            guess_q, config_sol);
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

    swing_leg_traj_.SetParam(init, fin, middle_pt, end_time_ - replan_moment_);

    delete [] *middle_pt;
    delete [] middle_pt;
}

void BodyJPosSwingPlanningCtrl::_SetBspline(const dynacore::Vect3 & st_pos,
        const dynacore::Vect3 & st_vel,
        const dynacore::Vect3 & st_acc,
        const dynacore::Vect3 & target_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];

    // printf("time (state/end): %f, %f\n", state_machine_time_, end_time_);
    double portion = (1./end_time_) * (end_time_/2. - state_machine_time_);
    // printf("portion: %f\n\n", portion);
    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = st_vel[i];
        init[i+6] = st_acc[i];
        // Final
        fin[i] = target_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;

        if(portion > 0.)
            middle_pt[0][i] = (st_pos[i] + target_pos[i])*portion;
        else
            middle_pt[0][i] = (st_pos[i] + target_pos[i])/2.;
    }
    if(portion > 0.)  middle_pt[0][2] = swing_height_ + st_pos[2];

    foot_traj_.SetParam(init, fin, middle_pt, end_time_ - replan_moment_);

    delete [] *middle_pt;
    delete [] middle_pt;
}

void BodyJPosSwingPlanningCtrl::LastVisit(){
}

bool BodyJPosSwingPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        //printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
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
            printf("[Body Foot JPos Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);

            return true;
        }
    }
    return false;
}

void BodyJPosSwingPlanningCtrl::_setTaskGain(
        const dynacore::Vector & Kp, const dynacore::Vector & Kd){
    ((ConfigTask*)config_body_foot_task_)->Kp_vec_ = Kp;
    ((ConfigTask*)config_body_foot_task_)->Kd_vec_ = Kd;
}

void BodyJPosSwingPlanningCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);
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
        task_kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        task_kd_[i] = tmp_vec[i];
    }

    ((ConfigTask*)config_body_foot_task_)->Kp_vec_ = task_kp_;
    ((ConfigTask*)config_body_foot_task_)->Kd_vec_ = task_kd_;

    // Feedback gain decreasing near to the landing moment
    handler.getValue("gain_decreasing_ratio", gain_decreasing_ratio_);
    handler.getValue("gain_decreasing_period_portion", 
            gain_decreasing_period_portion_);

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }

    handler.getBoolean("initial_planning", b_initial_planning_);
    handler.getValue("kp_x", kp_x_);
    handler.getValue("kp_y", kp_y_);

    handler.getValue("swing_time_reduction", swing_time_reduction_);

    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_- swing_time_reduction_);
        b_bodypute_eigenvalue = false;
    }
    //printf("[Body Foot JPos Planning Ctrl] Parameter Setup Completed\n");
}
