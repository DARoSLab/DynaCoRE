#include "JPosTargetCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* robot):Controller(robot),
    jpos_target_(mercury::num_act_joint),
    jpos_ini_(mercury::num_act_joint),
    end_time_(1000.0),
    b_external_initial_pos_set_(false),
    ctrl_start_time_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint)
{
    jpos_task_ = new JPosTask();
    fixed_body_contact_ = new FixedBodyContact(robot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);

    wbdc_rotor_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 1.0);

    sp_ = Mercury_StateProvider::getStateProvider();
}

JPosTargetCtrl::~JPosTargetCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosTargetCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;
    _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void JPosTargetCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }

    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
}


void JPosTargetCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(mercury::num_act_joint); jacc_des.setZero();

    for(int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = dynacore::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        des_jvel_[i] = dynacore::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        jacc_des[i] = dynacore::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
    }

    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, jacc_des);
    task_list_.push_back(jpos_task_);
}

void JPosTargetCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosTargetCtrl::setInitialPosition(const std::vector<double> & jpos_ini){
    for(int i(0); i<mercury::num_act_joint; ++i){
        jpos_ini_[i] = jpos_ini[i];
    }
    b_external_initial_pos_set_ = true;
}
void JPosTargetCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
    if(!b_external_initial_pos_set_)
        jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
}

void JPosTargetCtrl::LastVisit(){
}

bool JPosTargetCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosTargetCtrl::CtrlInitialization(const std::string & setting_file_name){
    if(!b_external_initial_pos_set_)
        jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];
    }

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((JPosTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }

    //printf("JPos Target Ctrl Param Set!\n");
}

void JPosTargetCtrl::setTargetPosition(const std::vector<double>& jpos){
    for(int i(0); i<mercury::num_act_joint; ++i){
        jpos_target_[i] = jpos[i];
         //printf("%i th jpos: %f\n", i, jpos[i]);
    }
}
