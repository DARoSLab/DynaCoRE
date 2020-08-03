#include "JPosTrajPlanningCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/WBWC.hpp>

#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

JPosTrajPlanningCtrl::JPosTrajPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    push_down_height_(0.0),
    swing_jpos_delta_(3),
    prev_jpos_des_(mercury::num_act_joint),
    target_swing_leg_config_(3)
{
    prev_jpos_des_.setZero();
    swing_jpos_delta_.setZero();
    des_jacc_.setZero();

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 1000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;
 
    if(swing_foot == mercury_link::leftFoot) {
        swing_leg_jidx_ = mercury_joint::leftAbduction;
        stance_leg_jidx_ = mercury_joint::rightAbduction;

        wbwc_->W_rf_[3] = 5.0;
        wbwc_->W_rf_[4] = 5.0;
        wbwc_->W_rf_[5] = 0.5;

        wbwc_->W_foot_[3] = 0.001;
        wbwc_->W_foot_[4] = 0.001;
        wbwc_->W_foot_[5] = 0.001;

        wbwc_->left_z_max_ = 0.0001; 
    }
    else if(swing_foot == mercury_link::rightFoot) {
        swing_leg_jidx_ = mercury_joint::rightAbduction;
        stance_leg_jidx_ = mercury_joint::leftAbduction;

        wbwc_->W_rf_[0] = 5.0;
        wbwc_->W_rf_[1] = 5.0;
        wbwc_->W_rf_[2] = 0.5;

        wbwc_->W_foot_[0] = 0.001;
        wbwc_->W_foot_[1] = 0.001;
        wbwc_->W_foot_[2] = 0.001;

        wbwc_->right_z_max_ = 0.0001; 
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    for(size_t i = 0; i < 3; i++){
        min_jerk_jpos_.push_back(new MinJerk_OneDimension());
    }
    printf("[BodyFootJPosPlanning Controller] Constructed\n");
}
void JPosTrajPlanningCtrl::_SetMinJerkTraj(
        double moving_duration,
        const dynacore::Vect3 & st_pos,
        const dynacore::Vect3 & st_vel,
        const dynacore::Vect3 & st_acc,
        const dynacore::Vect3 & target_pos,
        const dynacore::Vect3 & target_vel,
        const dynacore::Vect3 & target_acc){

    std::vector< dynacore::Vect3 > min_jerk_initial_params;
    std::vector< dynacore::Vect3 > min_jerk_final_params;   
    dynacore::Vect3 init_params; 
    dynacore::Vect3 final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        init_params.setZero(); final_params.setZero();
        // Set Dimension i's initial pos, vel and acceleration
        init_params[0] = st_pos[i]; init_params[1] = st_vel[i];  init_params[2] = st_acc[i];
        // Set Dimension i's final pos, vel, acceleration
        final_params[0] = target_pos[i]; final_params[1] = target_vel[i];  final_params[2] = target_acc[i];
        // Add to the parameter vector 
        min_jerk_initial_params.push_back(init_params);
        min_jerk_final_params.push_back(final_params);
    }


    // Set Minimum Jerk Parameters for each dimension
    for(size_t i = 0; i < 3; i++){
        min_jerk_jpos_[i]->setParams(
                min_jerk_initial_params[i], 
                min_jerk_final_params[i],
                0., moving_duration);  
        // min_jerk_cartesian[i]->printParameters();  
    }
}

JPosTrajPlanningCtrl::~JPosTrajPlanningCtrl(){
    delete wbwc_;
}

void JPosTrajPlanningCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _task_setup();
    _body_foot_ctrl(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    sp_->curr_jpos_des_ = des_jpos_;
    _PostProcessing_Command();
}
void JPosTrajPlanningCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<mercury::num_act_joint; ++i){
        A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    wbwc_->UpdateSetting(A_rotor, coriolis_, grav_);
    wbwc_->computeTorque(des_jpos_, des_jvel_, des_jacc_, gamma);

    sp_->qddot_cmd_ = wbwc_->qddot_;
    sp_->reaction_forces_ = wbwc_->Fr_;
}


void JPosTrajPlanningCtrl::_task_setup(){
    // Body height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();

    // TEST
    rpy_des[1] = sp_->des_body_pitch_;
    dynacore::convert(rpy_des, des_quat);

    _CoMEstiamtorUpdate();
    _CheckPlanning();

    double traj_time = state_machine_time_ - half_swing_time_;

    dynacore::Vector config_sol, qdot_cmd, qddot_cmd;
    inv_kin_.getSingleSupportFullConfigSeperation(
            sp_->Q_, des_quat, target_height, 
            swing_foot_, curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_,
            config_sol, qdot_cmd, qddot_cmd);

    double amp;
    double omega(2.*M_PI/end_time_);

    for(int i(0); i<3; ++i){
        amp = swing_jpos_delta_[i]/2.;
        config_sol[swing_leg_jidx_ + i] = 
            prev_jpos_des_[swing_leg_jidx_ + i - mercury::num_virtual]
            + amp * (1 - cos(omega * state_machine_time_));
        qdot_cmd[swing_leg_jidx_ + i] = 
            amp * omega * sin(omega * state_machine_time_);
        qddot_cmd[swing_leg_jidx_ + i] = 
            amp * omega * omega * cos(omega * state_machine_time_);
    }

    if(state_machine_time_ > half_swing_time_){
        double pos, vel, acc;
        for(int i(0); i<3; ++i){
            min_jerk_jpos_[i]->getPos(traj_time, pos);  
            min_jerk_jpos_[i]->getVel(traj_time, vel);
            min_jerk_jpos_[i]->getAcc(traj_time, acc);
            
            config_sol[swing_leg_jidx_ + i] += pos;
            qdot_cmd[swing_leg_jidx_ + i] += vel;
            qddot_cmd[swing_leg_jidx_ + i] += acc;
        }

        double ramp_time(0.05);
        // TEST
        double alpha(1.);
        if(traj_time < ramp_time){
            alpha = traj_time/ramp_time;
        }
        config_sol[swing_leg_jidx_] += alpha*roll_offset_gain_ * sp_->Q_[3];
        config_sol[swing_leg_jidx_+ 1] += alpha*pitch_offset_gain_ * sp_->Q_[4];
    }
    // CAREFULL!!!!!!!!!!!!!!!!!!!!!
    // TEST 
   // config_sol[stance_leg_jidx_+ 2] -= 0.05; 

    for(int i(0); i<mercury::num_act_joint; ++i){
        sp_->jacc_des_[i] = qddot_cmd[i + mercury::num_virtual];
    }

    for (int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = config_sol[mercury::num_virtual + i];
        des_jvel_[i] = qdot_cmd[mercury::num_virtual + i];
        des_jacc_[i] = qddot_cmd[mercury::num_virtual + i];
    }

    // TEST
    des_jacc_.setZero();

}

void JPosTrajPlanningCtrl::_CheckPlanning(){
    if( state_machine_time_ > 
            (end_time_/(planning_frequency_ + 1.) * (num_planning_ + 1.) + 0.002) ){
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);

        curr_foot_pos_des_ = target_loc;
        dynacore::Vector guess_q = sp_->Q_;
        // dynacore::Vector guess_q = sp_->jjpos_config_;
        dynacore::Vector config_sol = sp_->Q_;

        inv_kin_.getLegConfigAtVerticalPosture(
            swing_foot_, target_loc, guess_q, config_sol);
        target_swing_leg_config_ = config_sol.segment(swing_leg_jidx_, 3);

        adjust_jpos_ = 
            target_swing_leg_config_ 
            - prev_jpos_des_.segment(swing_leg_jidx_ - mercury::num_virtual, 3);

        dynacore::Vect3 zero; zero.setZero();

        _SetMinJerkTraj(half_swing_time_, 
                zero, zero, zero,
                adjust_jpos_, zero, zero);


        ++num_planning_;
    }
}

void JPosTrajPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){
    dynacore::Vect3 com_pos, com_vel;
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
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        //com_pos[i] += body_pt_offset_[i];
        // com_vel[i] = sp_->average_vel_[i]; 
        // com_vel[i] = sp_->est_CoM_vel_[i]; 

        // com_pos[i] = sp_->est_mocap_body_pos_[i] + body_pt_offset_[i];
        // com_vel[i] = sp_->est_mocap_body_vel_[i]; 
        
        //com_vel[i] = sp_->ekf_body_vel_[i]; 
    }

    printf("planning com state: %f, %f, %f, %f\n",
            com_pos[0], com_pos[1],
            com_vel[0], com_vel[1]);

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
    target_loc -= sp_->global_pos_local_;

    target_loc[2] = default_target_loc_[2];
    target_loc[2] -= push_down_height_;
    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}

void JPosTrajPlanningCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;
    num_planning_ = 0;
    prev_jpos_des_ = sp_->curr_jpos_des_;

    robot_sys_->getPos(swing_foot_, curr_foot_pos_des_);

    dynacore::Vector config_sol;
    dynacore::Vect3 target_loc = default_target_loc_; 

    target_loc[0] = sp_->Q_[0];
    target_loc[1] = sp_->Q_[1] + default_target_loc_[1];

    // Compute Target config
    inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, target_loc, 
            sp_->Q_, config_sol);
    target_swing_leg_config_ = config_sol.segment(swing_leg_jidx_, 3);

    adjust_jpos_ = 
        target_swing_leg_config_ 
        - prev_jpos_des_.segment(swing_leg_jidx_ - mercury::num_virtual, 3);

    dynacore::Vect3 zero; zero.setZero();

    _SetMinJerkTraj(half_swing_time_, 
            zero, zero, zero,
            adjust_jpos_, zero, zero);

    dynacore::Vect3 com_vel;
    robot_sys_->getCoMPosition(ini_com_pos_);
    robot_sys_->getCoMVelocity(com_vel);
    dynacore::Vector input_state(4);
    input_state[0] = ini_com_pos_[0];   
    input_state[1] = ini_com_pos_[1];
    input_state[2] = com_vel[0];   input_state[3] = com_vel[1];

    com_estimator_->EstimatorInitialization(input_state);
    _CoMEstiamtorUpdate();
}

bool JPosTrajPlanningCtrl::EndOfPhase(){
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

void JPosTrajPlanningCtrl::CtrlInitialization(const std::string & setting_file_name){
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
    for(int i(0); i<mercury::num_act_joint; ++i){
        wbwc_->Kp_[i] = tmp_vec[i + mercury::num_virtual];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<mercury::num_act_joint; ++i){
        wbwc_->Kd_[i] = tmp_vec[i + mercury::num_virtual];
    }

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }
    handler.getVector("swing_jpos_delta", tmp_vec);
    for(int i(0); i<3; ++i){ swing_jpos_delta_[i] = tmp_vec[i]; }

    // Foot landing offset
    handler.getValue("roll_offset_gain", roll_offset_gain_);
    handler.getValue("pitch_offset_gain", pitch_offset_gain_);

    static bool b_bodypute_eigenvalue(true);
    if(b_bodypute_eigenvalue){
        ((Reversal_LIPM_Planner*)planner_)->
            CheckEigenValues(double_stance_ratio_*stance_time_ + 
                    transition_phase_ratio_*transition_time_ + 
                    end_time_);
        b_bodypute_eigenvalue = false;
    }
}
