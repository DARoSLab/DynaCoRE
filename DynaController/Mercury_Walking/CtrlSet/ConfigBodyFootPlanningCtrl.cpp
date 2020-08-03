#include "ConfigBodyFootPlanningCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <Mercury_Controller/WBWC.hpp>


ConfigBodyFootPlanningCtrl::ConfigBodyFootPlanningCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    SwingPlanningCtrl(robot, swing_foot, planner),
    push_down_height_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    waiting_time_limit_(0.02)
{
    des_jacc_.setZero();

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 1000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;
 
    if(swing_foot == mercury_link::leftFoot) {
        wbwc_->W_rf_[3] = 5.0;
        wbwc_->W_rf_[4] = 5.0;
        wbwc_->W_rf_[5] = 0.5;

        wbwc_->W_foot_[3] = 0.001;
        wbwc_->W_foot_[4] = 0.001;
        wbwc_->W_foot_[5] = 0.001;

        wbwc_->left_z_max_ = 0.0001; 
    }
    else if(swing_foot == mercury_link::rightFoot) {
        wbwc_->W_rf_[0] = 5.0;
        wbwc_->W_rf_[1] = 5.0;
        wbwc_->W_rf_[2] = 0.5;

        wbwc_->W_foot_[0] = 0.001;
        wbwc_->W_foot_[1] = 0.001;
        wbwc_->W_foot_[2] = 0.001;

        wbwc_->right_z_max_ = 0.0001; 
    }
    else printf("[Warnning] swing foot is not foot: %i\n", swing_foot);

    // Create Minimum jerk objects
    for(size_t i = 0; i < 3; i++){
        min_jerk_offset_.push_back(new MinJerk_OneDimension());
    }
    printf("[Configuration BodyFootPlanning Controller] Constructed\n");
}

void ConfigBodyFootPlanningCtrl::OneStep(void* _cmd){   
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

void ConfigBodyFootPlanningCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
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

void ConfigBodyFootPlanningCtrl::_GetSinusoidalSwingTrajectory(){
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

void ConfigBodyFootPlanningCtrl::_GetBsplineSwingTrajectory(){
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
void ConfigBodyFootPlanningCtrl::_SetBspline(
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

void ConfigBodyFootPlanningCtrl::_task_setup(){
    // Body height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    rpy_des[1] = sp_->des_body_pitch_;
    dynacore::convert(rpy_des, des_quat);

    _CheckPlanning();
    // _GetSinusoidalSwingTrajectory();
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
        for(int i(0); i<3; ++i){
            curr_foot_vel_des_[i] = 0;
            curr_foot_acc_des_[i] = 0;
        }
        curr_foot_pos_des_[2] = 
          -push_down_height_ - 0.1*(state_machine_time_ - end_time_);
    }
 
    dynacore::Vector config_sol, qdot_cmd, qddot_cmd;
    inv_kin_.getSingleSupportFullConfigSeperation(
            sp_->Q_, des_quat, target_height, 
            swing_foot_, 
            curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_,
            config_sol, qdot_cmd, qddot_cmd);

    for (int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = config_sol[mercury::num_virtual + i];
        des_jvel_[i] = qdot_cmd[mercury::num_virtual + i];
        des_jacc_[i] = qddot_cmd[mercury::num_virtual + i];
    }
}

void ConfigBodyFootPlanningCtrl::_CheckPlanning(){
    if( state_machine_time_ > 
            (end_time_/(planning_frequency_ + 1.) * (num_planning_ + 1.) + 0.002) ){
        if(num_planning_<planning_frequency_){
            dynacore::Vect3 target_loc;
            _Replanning(target_loc);

            dynacore::Vect3 target_offset;
            // X, Y target is originally set by intial_traget_loc
            for(int i(0); i<2; ++i)
                target_offset[i] = target_loc[i] - initial_target_loc_[i];

            // Foot height (z) is set by the initial height
            target_offset[2] = 0.; //target_loc[2] - ini_foot_pos_[2];

            _SetMinJerkOffset(target_offset);
            ++num_planning_;

        }
    }
}

void ConfigBodyFootPlanningCtrl::_Replanning(dynacore::Vect3 & target_loc){
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
        com_pos[i] = sp_->Q_[i] + body_pt_offset_[i];
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        // com_pos[i] += body_pt_offset_[i];
        // com_vel[i] = sp_->average_vel_[i]; 
        // com_vel[i] = sp_->est_CoM_vel_[i]; 

         // com_pos[i] = sp_->est_mocap_body_pos_[i] + body_pt_offset_[i];
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        com_vel[i] = sp_->est_mocap_body_vel_[i]; 
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

    
    dynacore::Vect3 tmp_global_pos_local = sp_->global_pos_local_;

    planner_->getNextFootLocation(com_pos + tmp_global_pos_local,
            com_vel,
            target_loc,
            &pl_param, &pl_output);
    // Time Modification
    replan_moment_ = state_machine_time_;
    end_time_ += pl_output.time_modification;
    target_loc -= sp_->global_pos_local_;
 
    target_loc[2] = initial_target_loc_[2];
    // target_loc[2] -= push_down_height_;

    // TEST
    for(int i(0); i<2; ++i){
        target_loc[i] += foot_landing_offset_[i];
    }
    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}

void ConfigBodyFootPlanningCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;
    num_planning_ = 0;

    // TEST
    initial_target_loc_[0] = sp_->Q_[0];
    initial_target_loc_[1] = sp_->Q_[1] + default_target_loc_[1];
    initial_target_loc_[2] = -push_down_height_; //ini_foot_pos_[2];

    _SetBspline(ini_foot_pos_, initial_target_loc_);

    dynacore::Vect3 foot_pos_offset; foot_pos_offset.setZero();
    foot_pos_offset[2] = 0.;// - ini_foot_pos_[2];
    _SetMinJerkOffset(foot_pos_offset);

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

void ConfigBodyFootPlanningCtrl::_SetMinJerkOffset(const dynacore::Vect3 & offset){
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

bool ConfigBodyFootPlanningCtrl::EndOfPhase(){
    if(state_machine_time_ > (end_time_ + waiting_time_limit_)){
        printf("[Body Foot Ctrl] End, state_machine time/ end time: (%f, %f)\n", 
            state_machine_time_, end_time_);
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
            printf("[Config Body Foot Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);
            return true;
        }
    }
    return false;
}

void ConfigBodyFootPlanningCtrl::CtrlInitialization(const std::string & setting_file_name){
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

ConfigBodyFootPlanningCtrl::~ConfigBodyFootPlanningCtrl(){
    delete wbwc_;
}


