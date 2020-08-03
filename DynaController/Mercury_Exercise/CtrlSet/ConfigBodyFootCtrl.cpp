#include "ConfigBodyFootCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/SingleContact.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/utilities.hpp>

#define MEASURE_TIME_WBDC 0

#if MEASURE_TIME_WBDC
#include <chrono>
#endif 

ConfigBodyFootCtrl::ConfigBodyFootCtrl(RobotSystem* robot, int swing_foot):
    Controller(robot),
    swing_foot_(swing_foot),
    ctrl_start_time_(0.),
    moving_time_(1.),
    swing_time_(1000.)
{
    curr_foot_pos_des_.setZero();
    curr_foot_vel_des_.setZero();
    curr_foot_acc_des_.setZero();

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

    wbdc_rotor_data_->cost_weight[0] = 200;    
    wbdc_rotor_data_->cost_weight[1] = 200;    
    wbdc_rotor_data_->cost_weight[2] = 200;    
    wbdc_rotor_data_->cost_weight[3] = 200;    
    
    wbdc_rotor_data_->cost_weight.tail(single_contact_->getDim()) = 
        dynacore::Vector::Constant(single_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[config_body_foot_task_->getDim() + 2]  = 0.001; // Fr_z

    sp_ = Mercury_StateProvider::getStateProvider();
    printf("[BodyFootJPos Controller] Constructed\n");
}

ConfigBodyFootCtrl::~ConfigBodyFootCtrl(){
    delete config_body_foot_task_;
    delete single_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void ConfigBodyFootCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _single_contact_setup();
    _task_setup();
    _body_foot_ctrl(gamma);

    _PostProcessing_Command();
}

void ConfigBodyFootCtrl::_body_foot_ctrl(dynacore::Vector & gamma){
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

void ConfigBodyFootCtrl::_task_setup(){
    dynacore::Vector pos_des(config_body_foot_task_->getDim());
    dynacore::Vector vel_des(config_body_foot_task_->getDim());
    dynacore::Vector acc_des(config_body_foot_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body Pos
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);

    // Foot Position Task
    double pos[3];
    double vel[3];
    double acc[3];
    double traj_time = state_machine_time_;

    if(state_machine_time_ > moving_time_){
        traj_time = state_machine_time_ - moving_time_;
        double omega;
        for(int i(0); i<3; ++i){
            omega = 2. * M_PI * freq_[i];
            pos[i] = 
                foot_pos_set_[i] + amp_[i] * sin(omega * traj_time + phase_[i]);
            vel[i] =  
                amp_[i] * omega * cos(omega * traj_time + phase_[i]);
            acc[i] = 
                -amp_[i] * omega * omega * sin(omega * traj_time + phase_[i]);
        }
    }else {
        foot_traj_.getCurvePoint(traj_time, pos);
        foot_traj_.getCurveDerPoint(traj_time, 1, vel);
        foot_traj_.getCurveDerPoint(traj_time, 2, acc);
    }

    // Foot position setup (and For save)
    for(int i(0); i<3; ++i){
        curr_foot_pos_des_[i] = pos[i];
        curr_foot_vel_des_[i] = vel[i];
        curr_foot_acc_des_[i] = acc[i];
    }
  
    dynacore::Vector config_sol, qdot_cmd, qddot_cmd;
    // inv_kin_.getSingleSupportFullConfig(
    //         sp_->Q_, des_quat, target_height, 
    //         swing_foot_, curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_,
    //         config_sol, qdot_cmd, qddot_cmd);

    inv_kin_.getSingleSupportFullConfigSeperation(
            sp_->Q_, des_quat, target_height, 
            swing_foot_, curr_foot_pos_des_, curr_foot_vel_des_, curr_foot_acc_des_,
            config_sol, qdot_cmd, qddot_cmd);
    
    for (int i(0); i<mercury::num_act_joint; ++i){
        pos_des[mercury::num_virtual + i] = config_sol[mercury::num_virtual + i];  
        vel_des[mercury::num_virtual + i] = qdot_cmd[mercury::num_virtual + i];
        acc_des[mercury::num_virtual + i] = qddot_cmd[mercury::num_virtual + i];
        sp_->jpos_des_[i] = pos_des[mercury::num_virtual + i];
        sp_->jvel_des_[i] = vel_des[mercury::num_virtual + i];
    }

    // dynacore::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
    // Push back to task list
    config_body_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(config_body_foot_task_);
}

void ConfigBodyFootCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void ConfigBodyFootCtrl::FirstVisit(){
    robot_sys_->getPos(swing_foot_, ini_foot_pos_);
    robot_sys_->getPos(swing_foot_, foot_pos_set_);
    ctrl_start_time_ = sp_->curr_time_;
    state_machine_time_ = 0.;

    dynacore::Vect3 zero;
    zero.setZero();
    foot_pos_set_[2] += swing_height_;
    dynacore::Vect3 foot_swing_vel_ini;

    for(int i(0); i<3; ++i){
        foot_swing_vel_ini[i] = 2. * M_PI * freq_[i] * amp_[i];
    }

    _SetBspline(ini_foot_pos_, zero, zero, 
            foot_pos_set_, 
            foot_swing_vel_ini,
            zero);

    //dynacore::pretty_print(ini_swing_leg_config_, std::cout, "ini leg config");
}

void ConfigBodyFootCtrl::_SetBspline(const dynacore::Vect3 & st_pos,
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


void ConfigBodyFootCtrl::LastVisit(){
}

bool ConfigBodyFootCtrl::EndOfPhase(){
    if(state_machine_time_ > moving_time_ + swing_time_){
        return true;
    } 
    return false;
}

void ConfigBodyFootCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);
    std::vector<double> tmp_vec;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_body_foot_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_body_foot_task_)->Kd_vec_[i] = tmp_vec[i];
    }
    //printf("[Body Foot Ctrl] Parameter Setup Completed\n");
}
