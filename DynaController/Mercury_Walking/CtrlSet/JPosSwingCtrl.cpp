#include "JPosSwingCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>

#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>

#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury_Controller/WBWC.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>


#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

JPosSwingCtrl::JPosSwingCtrl(
        const RobotSystem* robot, int swing_foot, Planner* planner):
    Controller(robot),
    swing_foot_(swing_foot),
    planner_(planner),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
    jpos_swing_delta_(3),
    stance_jpos_(mercury::num_act_joint),
    planning_moment_portion_(0.5),
    push_down_height_(0.0),
    abduction_addition_(0.),
    hip_addition_(0.),
    abduction_addition_replan_(0.),
    hip_addition_replan_(0.),
    b_replan_(false),
    b_planned_(false),
    prev_jpos_des_(mercury::num_act_joint),
    target_swing_leg_config_(3),
    swing_jpos_addition_(3),
    swing_jpos_replanned_addition_(3)
{
    //jpos_task_ = new JPosTask();
    //contact_constraint_ = new FixedBodyContact(robot);
    default_target_loc_.setZero();
    prev_jpos_des_.setZero();
    swing_jpos_addition_.setZero();

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 10000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;

    jpos_task_ = new ConfigTask();

    curr_foot_pos_des_.setZero();
    curr_foot_vel_des_.setZero();
    curr_foot_acc_des_.setZero();

    if(swing_foot_ == mercury_link::leftFoot) {
        swing_leg_jidx_ = mercury_joint::leftAbduction - mercury::num_virtual;
        stance_leg_jidx_ = mercury_joint::rightAbduction - mercury::num_virtual;

        contact_constraint_ = new SingleContact(robot, mercury_link::rightFoot);

        wbwc_->W_rf_[3] = 5.0;
        wbwc_->W_rf_[4] = 5.0;
        wbwc_->W_rf_[5] = 0.5;

        wbwc_->W_foot_[3] = 0.001;
        wbwc_->W_foot_[4] = 0.001;
        wbwc_->W_foot_[5] = 0.001;

        wbwc_->left_z_max_ = 0.0001; 
    }
    else if(swing_foot_ == mercury_link::rightFoot) {
        swing_leg_jidx_ = mercury_joint::rightAbduction - mercury::num_virtual;
        stance_leg_jidx_ = mercury_joint::leftAbduction - mercury::num_virtual;

        contact_constraint_ = new SingleContact(robot, mercury_link::leftFoot);

        wbwc_->W_rf_[0] = 5.0;
        wbwc_->W_rf_[1] = 5.0;
        wbwc_->W_rf_[2] = 0.5;

        wbwc_->W_foot_[0] = 0.001;
        wbwc_->W_foot_[1] = 0.001;
        wbwc_->W_foot_[2] = 0.001;

        wbwc_->right_z_max_ = 0.0001; 
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
                jpos_task_->getDim() + 
                contact_constraint_->getDim(), 1000.0);

    
   //wbdc_rotor_data_->cost_weight.head(2) = 
        //dynacore::Vector::Constant(2, 0.1);

    wbdc_rotor_data_->cost_weight.tail(contact_constraint_->getDim()) = 
        dynacore::Vector::Constant(contact_constraint_->getDim(), 0.1);
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;

    //wbdc_rotor_data_->cost_weight.head(mercury::num_virtual) 
        //= dynacore::Vector::Constant(mercury::num_virtual, 0.001);
    //wbdc_rotor_data_->cost_weight.segment(mercury::num_virtual, mercury::num_act_joint) 
        //= dynacore::Vector::Constant(mercury::num_act_joint, 100000.0);
    sp_ = Mercury_StateProvider::getStateProvider();

    for(size_t i = 0; i < 3; i++){
        min_jerk_jpos_initial_.push_back(new MinJerk_OneDimension());
    }
    printf("[BodyFootJPosPlanning Controller] Constructed\n");
}
void JPosSwingCtrl::_SetMinJerkTraj(
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
        min_jerk_jpos_initial_[i]->setParams(
                min_jerk_initial_params[i], 
                min_jerk_final_params[i],
                0., moving_duration);  
        // min_jerk_cartesian[i]->printParameters();  
    }
}


JPosSwingCtrl::~JPosSwingCtrl(){
    delete jpos_task_;
    delete contact_constraint_;
}

void JPosSwingCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma;
    _contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}
void JPosSwingCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    //// WBDC
    //
    //for (int i(0); i<mercury::num_act_joint; ++i){
        //wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            //= sp_->rotor_inertia_[i];
    //}
    //wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    //wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    //gamma = wbdc_rotor_data_->cmd_ff;

    //int offset(0);
    //if(swing_foot_ == mercury_link::rightFoot) offset = 3;
    //dynacore::Vector reaction_force = 
        //(wbdc_rotor_data_->opt_result_).tail(contact_constraint_->getDim());
    //for(int i(0); i<3; ++i)
        //sp_->reaction_forces_[i + offset] = reaction_force[i];

    //sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;

    // WBWC
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


void JPosSwingCtrl::_task_setup(){
    des_jvel_.setZero(); des_jacc_.setZero();
    des_jpos_ = stance_jpos_;

    double amp;
    double omega(2.*M_PI/end_time_);

    if(b_replan_)   _CheckPlanning();

    for(int i(0); i<3; ++i){
        amp = jpos_swing_delta_[i]/2.;
 
        sp_->curr_jpos_des_[swing_leg_jidx_ + i] = 
            prev_jpos_des_[swing_leg_jidx_ + i]
            + amp * (1 - cos(omega * state_machine_time_))
            + dynacore::smooth_changing(
                    0., swing_jpos_addition_[i], end_time_, state_machine_time_);
        des_jvel_[swing_leg_jidx_ + i] = 
            amp * omega * sin(omega * state_machine_time_)
            + dynacore::smooth_changing_vel(
                    0., swing_jpos_addition_[i], end_time_, state_machine_time_);
         des_jacc_[swing_leg_jidx_ + i] = 
            amp * omega * omega * cos(omega * state_machine_time_)
             + dynacore::smooth_changing_acc(
                    0., swing_jpos_addition_[i], end_time_, state_machine_time_);
       
        //des_jpos_[swing_leg_jidx_ + i] += amp * (1 - cos(omega * state_machine_time_));
        //des_jvel_[swing_leg_jidx_ + i] = amp * omega * sin(omega * state_machine_time_);
        //des_jacc_[swing_leg_jidx_ + i] = amp * omega * omega * cos(omega * state_machine_time_);
    }

    if((state_machine_time_ > planning_moment_portion_ * end_time_) && b_replan_){
        double time_after(state_machine_time_ - planning_moment_portion_ * end_time_);
        double remain_duration(end_time_ * (1-planning_moment_portion_));

        for(int i(0); i<3; ++i){
            sp_->curr_jpos_des_[swing_leg_jidx_ + i] += 
                dynacore::smooth_changing(0., swing_jpos_replanned_addition_[i], 
                        remain_duration, time_after);

            des_jvel_[swing_leg_jidx_ + i] += 
                dynacore::smooth_changing_vel(0., swing_jpos_replanned_addition_[i], 
                        remain_duration, time_after);
        }
        sp_->curr_jpos_des_[swing_leg_jidx_] += 
            roll_offset_gain_ *  sp_->Q_[mercury_joint::virtual_Rx];
        sp_->curr_jpos_des_[swing_leg_jidx_ + 1] += 
            pitch_offset_gain_ *  sp_->Q_[mercury_joint::virtual_Ry];
    }
    // Abduction roll
    sp_->curr_jpos_des_[stance_leg_jidx_] += 
        sp_->Kp_roll_ * sp_->Q_[mercury_joint::virtual_Rx];
    des_jacc_[stance_leg_jidx_] = 
        sp_->Kd_roll_ * sp_->Q_[mercury_joint::virtual_Rx];
    // Hip Pitch
    sp_->curr_jpos_des_[stance_leg_jidx_ + 1] += 
        sp_->Kp_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
    des_jacc_[stance_leg_jidx_ + 1] = 
        sp_->Kd_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
 
    des_jpos_ = sp_->curr_jpos_des_;

    // Configuration Setting
    dynacore::Vector config_des = sp_->Q_;
    config_des.segment(mercury::num_virtual, mercury::num_act_joint) = 
        des_jpos_;
    dynacore::Vector qdot_des(mercury::num_qdot);
    qdot_des.setZero();
    qdot_des.tail(mercury::num_act_joint) = des_jvel_;

    dynacore::Vector qddot_des(mercury::num_qdot);
    qddot_des.setZero();
    qddot_des.tail(mercury::num_act_joint) = des_jacc_;

   
    // Push back to task list
    //jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, des_jacc_);
    jpos_task_->UpdateTask(&(config_des), qdot_des, qddot_des);
    task_list_.push_back(jpos_task_);
}

void JPosSwingCtrl::_CheckPlanning(){
    if( (state_machine_time_ > planning_moment_portion_ * end_time_) && !b_planned_ ){
        dynacore::Vect3 target_loc;
        _Replanning(target_loc);
        b_planned_ = true;
    }
}

void JPosSwingCtrl::_Replanning(dynacore::Vect3 & target_loc){
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
        // TEST jpos update must be true
        // com_pos[i] = sp_->jjpos_body_pos_[i] + body_pt_offset_[i];
        //com_pos[i] += body_pt_offset_[i];
        // com_vel[i] = sp_->average_vel_[i]; 
        // com_vel[i] = sp_->est_CoM_vel_[i]; 

        com_pos[i] = sp_->est_mocap_body_pos_[i] + body_pt_offset_[i];
        com_vel[i] = sp_->est_mocap_body_vel_[i]; 
        
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
    
    dynacore::Vector config_sol = sp_->Q_;
    inv_kin_.getLegConfigAtVerticalPosture(
            swing_foot_, target_loc, sp_->Q_, config_sol);
 
    dynacore::Vector new_target_pos = 
        config_sol.segment(swing_leg_jidx_ + mercury::num_virtual, 3);

    swing_jpos_replanned_addition_ = 
        new_target_pos - target_swing_leg_config_;

    curr_foot_pos_des_ = target_loc;
    dynacore::pretty_print(target_loc, std::cout, "next foot loc");
}


void JPosSwingCtrl::_contact_setup(){
    contact_constraint_->UpdateContactSpec();
    contact_list_.push_back(contact_constraint_);
}

void JPosSwingCtrl::FirstVisit(){
    ini_config_ = sp_->Q_;
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;
    replan_moment_ = 0.;
    
    prev_jpos_des_ = sp_->curr_jpos_des_;

    dynacore::Vector config_sol = sp_->Q_;
    dynacore::Vector guess_q = sp_->Q_;

    noplan_landing_loc_.setZero();
    if(swing_foot_ == mercury_link::leftFoot){
        noplan_landing_loc_.head(2) = 
            sp_->Q_.head(2) + sp_->default_lfoot_loc_.head(2);
    }else{
        noplan_landing_loc_.head(2) = 
            sp_->Q_.head(2) + sp_->default_rfoot_loc_.head(2);
    }
    inv_kin_.getLegConfigAtVerticalPosture(
            swing_foot_, noplan_landing_loc_, guess_q, config_sol);
 
    target_swing_leg_config_ = 
        config_sol.segment(mercury::num_virtual + swing_leg_jidx_, 3);

    swing_jpos_addition_ = 
        target_swing_leg_config_ - prev_jpos_des_.segment(swing_leg_jidx_, 3);

    //dynacore::pretty_print(config_sol, std::cout, "config sol");
    //dynacore::pretty_print(noplan_landing_loc_, std::cout, "landing loc");
    //dynacore::pretty_print(sp_->default_rfoot_loc_,std::cout, "defa rfoot");
    //dynacore::pretty_print(sp_->default_lfoot_loc_,std::cout, "defa lfoot");
    //dynacore::pretty_print(swing_jpos_addition_,std::cout, "jpos addition");
    //dynacore::pretty_print(target_swing_leg_config_,std::cout, "target");
    //dynacore::pretty_print(prev_jpos_des_,std::cout, "prev des jpos");
    //dynacore::pretty_print(guess_q,std::cout, "guess_q");

    b_planned_ = false;
}

void JPosSwingCtrl::LastVisit(){
}

bool JPosSwingCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
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
            printf("[Body Foot JPos Ctrl] contact happen, state_machine_time/ end time: (%f, %f)\n",
                    state_machine_time_, end_time_);

            return true;
        }
    }
    return false;
}


void JPosSwingCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("push_down_height", push_down_height_);

    handler.getVector("jpos_swing_delta", tmp_vec);
    for(int i(0); i<3; ++i){ jpos_swing_delta_[i] = tmp_vec[i]; }

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }
    wbwc_->Kp_ = ((JPosTask*)jpos_task_)->Kp_vec_.tail(mercury::num_act_joint);
    wbwc_->Kd_ = ((JPosTask*)jpos_task_)->Kd_vec_.tail(mercury::num_act_joint);

    // Body Point offset
    handler.getVector("body_pt_offset", tmp_vec);
    for(int i(0); i<2; ++i){
        body_pt_offset_[i] = tmp_vec[i];
    }

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


