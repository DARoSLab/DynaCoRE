#include "CoMFootCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/CoMFootTask.hpp>
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

CoMFootCtrl::CoMFootCtrl(RobotSystem* robot, int swing_foot):
    Controller(robot),
    swing_foot_(swing_foot),
    ctrl_start_time_(0.),
    moving_time_(1.),
    swing_time_(1000.)
{
    curr_foot_pos_des_.setZero();
    curr_foot_vel_des_.setZero();
    curr_foot_acc_des_.setZero();

    com_foot_task_ = new CoMFootTask(robot, swing_foot);
    if(swing_foot == mercury_link::leftFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::rightFoot); }
    else if(swing_foot == mercury_link::rightFoot) {
        single_contact_ = new SingleContact(robot, mercury_link::leftFoot); }
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


    sp_ = Mercury_StateProvider::getStateProvider();
     printf("[CoM Foot Controller] Constructed\n");
}

CoMFootCtrl::~CoMFootCtrl(){
    delete com_foot_task_;
    delete single_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void CoMFootCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _single_contact_setup();
    _task_setup();
    _com_foot_ctrl(gamma);

    _PostProcessing_Command();
}

void CoMFootCtrl::_com_foot_ctrl(dynacore::Vector & gamma){
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
}

void CoMFootCtrl::_task_setup(){
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

        pos_des[i + 7] = curr_foot_pos_des_[i];
        vel_des[i + 6] = curr_foot_vel_des_[i];
        acc_des[i + 6] = curr_foot_acc_des_[i];
    }
 
   // dynacore::pretty_print(vel_des, std::cout, "[Ctrl] vel des");
    // Push back to task list
    com_foot_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(com_foot_task_);
}

void CoMFootCtrl::_single_contact_setup(){
    single_contact_->UpdateContactSpec();
    contact_list_.push_back(single_contact_);
}

void CoMFootCtrl::FirstVisit(){
    // printf("[CoM Foot Ctrl] Start\n");

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

    // dynacore::pretty_print(ini_foot_pos_, std::cout, "ini foot pos");
}

void CoMFootCtrl::_SetBspline(const dynacore::Vect3 & st_pos,
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


void CoMFootCtrl::LastVisit(){
}

bool CoMFootCtrl::EndOfPhase(){
    if(state_machine_time_ > moving_time_ + swing_time_){
        return true;
    } 
    return false;
}

void CoMFootCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_com_pos_);
    std::vector<double> tmp_vec;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMFootTask*)com_foot_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMFootTask*)com_foot_task_)->Kd_vec_[i] = tmp_vec[i];
    }
    //printf("[CoM Foot Ctrl] Parameter Setup Completed\n");
}
