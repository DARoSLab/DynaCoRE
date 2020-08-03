#include "TransitionCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/BodyOriTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContactBounding.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

TransitionCtrl::TransitionCtrl(RobotSystem* robot, int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.)
{
    body_task_ = new BodyOriTask();
    double_contact_ = new DoubleContactBounding(robot, moving_foot);
    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(
                body_task_->getDim() + 
                double_contact_->getDim(), 100.0);

    wbdc_rotor_data_->cost_weight[0] = 0.0001; // X
    wbdc_rotor_data_->cost_weight[1] = 0.0001; // Y
    wbdc_rotor_data_->cost_weight[5] = 0.0001; // Yaw

    wbdc_rotor_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[body_task_->getDim() + 2]  = 0.001; // Fr_z
    wbdc_rotor_data_->cost_weight[body_task_->getDim() + 5]  = 0.001; // Fr_z

   sp_ = Mercury_StateProvider::getStateProvider();
     printf("[Transition Controller] Constructed\n");
}

TransitionCtrl::~TransitionCtrl(){
    delete body_task_;
    delete double_contact_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void TransitionCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    gamma.setZero();
    _double_contact_setup();
    _body_task_setup();
    _body_ctrl_wbdc_rotor(gamma);

    _PostProcessing_Command();
}

void TransitionCtrl::_body_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    
#if MEASURE_TIME_WBDC 
    static int time_count(0);
    time_count++;
    std::chrono::high_resolution_clock::time_point t1 
        = std::chrono::high_resolution_clock::now();
#endif
   gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2); 
    
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
        std::cout << "[body ctrl] WBDC_Rotor took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
    }
#endif
    
    dynacore::Vector reaction_force = 
        (wbdc_rotor_data_->opt_result_).tail(double_contact_->getDim());
    for(int i(0); i<double_contact_->getDim(); ++i)
        sp_->reaction_forces_[i] = reaction_force[i];

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
    //dynacore::pretty_print(sp_->reflected_reaction_force_, std::cout, "reflected force");
}

void TransitionCtrl::_body_task_setup(){
    dynacore::Vector pos_des(3 + 4);
    dynacore::Vector vel_des(body_task_->getDim());
    dynacore::Vector acc_des(body_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // CoM Pos
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

    body_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // Push back to task list
    task_list_.push_back(body_task_);
}

void TransitionCtrl::_double_contact_setup(){
    if(b_increase_){
        //((DoubleContactBounding*)double_contact_)->setFrictionCoeff(0.05, 0.3);
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            min_rf_z_ + state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    } else {
        //((DoubleContactBounding*)double_contact_)->setFrictionCoeff(0.3, 0.4);
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            max_rf_z_ - state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    }

    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void TransitionCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ctrl_start_time_ = sp_->curr_time_;
}

void TransitionCtrl::LastVisit(){
    // printf("[Transition] End\n");
}

bool TransitionCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void TransitionCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_body_pos_);
    std::vector<double> tmp_vec;

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((BodyOriTask*)body_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((BodyOriTask*)body_task_)->Kd_vec_[i] = tmp_vec[i];
    }

}
