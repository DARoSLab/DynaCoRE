#include "BodyFootJPosCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/BodyFootJPosTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/utilities.hpp>

#define MEASURE_TIME_WBDC 0

#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

BodyFootJPosCtrl::BodyFootJPosCtrl(RobotSystem* robot, int swing_foot):
    Controller(robot),
    swing_foot_(swing_foot),
    ctrl_start_time_(0.),
    moving_time_(1.),
    swing_time_(1000.)
{
    curr_foot_pos_des_.setZero();
    curr_foot_vel_des_.setZero();
    curr_foot_acc_des_.setZero();

    body_foot_task_ = new BodyFootJPosTask(swing_foot);
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
                body_foot_task_->getDim() + 
                single_contact_->getDim(), 1000.0);

    wbdc_rotor_data_->cost_weight[0] = 0.0001; // X
    wbdc_rotor_data_->cost_weight[1] = 0.0001; // Y
    wbdc_rotor_data_->cost_weight[5] = 0.0001; // Yaw

    wbdc_rotor_data_->cost_weight.tail(single_contact_->getDim()) = 
        dynacore::Vector::Constant(single_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[body_foot_task_->getDim() + 2]  = 0.001; // Fr_z


    sp_ = Mercury_StateProvider::getStateProvider();
     printf("[BodyFootJPos Controller] Constructed\n");
}

BodyFootJPosCtrl::~BodyFootJPosCtrl(){
    delete body_foot_task_;
    delete single_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void BodyFootJPosCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _single_contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    _PostProcessing_Command();
}

void BodyFootJPosCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
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
        std::cout << "[body foot jpos ctrl] WBDC_Rotor took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
    }
#endif

    int offset(0);
    if(swing_foot_ == mercury_link::rightFoot) offset = 3;
    dynacore::Vector reaction_force = 
        (wbdc_rotor_data_->opt_result_).tail(single_contact_->getDim());
    for(int i(0); i<3; ++i)
        sp_->reaction_forces_[i + offset] = reaction_force[i];
}

void BodyFootJPosCtrl::_task_setup(){
    dynacore::Vector pos_des(3 + 4 + 3);
    dynacore::Vector vel_des(body_foot_task_->getDim());
    dynacore::Vector acc_des(body_foot_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body Pos
    pos_des.head(3) = ini_body_pos_;
    if(b_set_height_target_) pos_des[2] = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();


    // Foot Position Task
    double pos[3];
    double vel[3];
    double acc[3];
    double traj_time = state_machine_time_;
    if(state_machine_time_ > moving_time_){
        traj_time = state_machine_time_ - moving_time_;
        double ramp(1.0);
        double ramping_time(0.8);
        if(traj_time < ramping_time){
            ramp = traj_time/ramping_time;
        }
        double omega;
        for(int i(0); i<3; ++i){
            omega = 2. * M_PI * freq_[i];
            pos[i] = target_swing_leg_config_[i] 
                + ramp * amp_[i] * sin(omega * traj_time + phase_[i]);
            vel[i] = ramp * amp_[i] * omega * cos(omega * traj_time + phase_[i]);
            acc[i] = -ramp * amp_[i] * omega * omega * sin(omega * traj_time + phase_[i]);
        }
    }else {
        for(int i(0); i<3; ++i){
            pos[i] = 
                dynacore::smooth_changing(
                        ini_swing_leg_config_[i], target_swing_leg_config_[i],
                        moving_time_, traj_time); 
            vel[i] =  
                dynacore::smooth_changing_vel(
                        ini_swing_leg_config_[i], target_swing_leg_config_[i],
                        moving_time_, traj_time); 
            acc[i] = 
                dynacore::smooth_changing_acc(
                        ini_swing_leg_config_[i], target_swing_leg_config_[i],
                        moving_time_, traj_time); 
        }
    }
    // Foot position setup (and For save)
    for(int i(0); i<3; ++i){
       
        sp_->jpos_des_[swing_leg_jidx_ - mercury::num_virtual + i] = pos[i];
        sp_->jvel_des_[swing_leg_jidx_ - mercury::num_virtual + i] = vel[i];
    
        pos_des[i + 7] = pos[i];
        vel_des[i + 6] = vel[i];
        acc_des[i + 6] = acc[i];
    }

    // dynacore::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
    // Push back to task list
    body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(body_foot_task_);
}

void BodyFootJPosCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void BodyFootJPosCtrl::FirstVisit(){
    ini_swing_leg_config_ = sp_->Q_.segment(swing_leg_jidx_, 3);
    dynacore::Vect3 target_foot_pos;
    robot_sys_->getPos(swing_foot_, target_foot_pos);
    target_foot_pos[2] += swing_height_;

    dynacore::Vector sol_config;
    inv_kin_.getLegConfigAtVerticalPosture(swing_foot_, target_foot_pos, 
            sp_->Q_, sol_config);
    target_swing_leg_config_ = sol_config.segment(swing_leg_jidx_, 3);

    //target_swing_leg_config_[1] -= 0.5;
    //target_swing_leg_config_[2] += 0.5;

    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;

     //dynacore::pretty_print(ini_swing_leg_config_, std::cout, "ini leg config");
}

void BodyFootJPosCtrl::_SetBspline(const dynacore::Vect3 & st_pos,
        const dynacore::Vect3 & st_vel,
        const dynacore::Vect3 & st_acc,
        const dynacore::Vect3 & target_pos, 
        const dynacore::Vect3 & target_vel,
        const dynacore::Vect3 & target_acc){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = st_vel[i];
        init[i+6] = st_acc[i];
        // Final
        fin[i] = target_pos[i];
        fin[i+3] = target_vel[i];
        fin[i+6] = target_acc[i];
    }
    foot_traj_.SetParam(init, fin, middle_pt, moving_time_);

    delete [] *middle_pt;
    delete [] middle_pt;
}


void BodyFootJPosCtrl::LastVisit(){
}

bool BodyFootJPosCtrl::EndOfPhase(){
    if(state_machine_time_ > moving_time_ + swing_time_){
        return true;
    } 
    return false;
}

void BodyFootJPosCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);
    std::vector<double> tmp_vec;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((BodyFootJPosTask*)body_foot_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((BodyFootJPosTask*)body_foot_task_)->Kd_vec_[i] = tmp_vec[i];
    }
    //printf("[Body Foot Ctrl] Parameter Setup Completed\n");
}
