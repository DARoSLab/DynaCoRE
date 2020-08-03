#include "JPosPostureFixCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>

#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>

#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/WBWC.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

JPosPostureFixCtrl::JPosPostureFixCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint),
   ctrl_start_time_(0.),
    set_jpos_(mercury::num_act_joint)
{
    set_jpos_.setZero();

    des_jacc_.setZero();
    //jpos_task_ = new JPosTask();
    //contact_constraint_ = new FixedBodyContact(robot);
    jpos_task_ = new ConfigTask();
    contact_constraint_ = new DoubleContact(robot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(contact_constraint_->getDim() + 
                jpos_task_->getDim(), 1000.0);
    wbdc_rotor_data_->cost_weight.tail(contact_constraint_->getDim()) = 
        dynacore::Vector::Constant(contact_constraint_->getDim(), 0.1);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    //wbdc_rotor_data_->cost_weight.head(mercury::num_virtual) 
        //= dynacore::Vector::Constant(mercury::num_virtual, 0.001);
    //wbdc_rotor_data_->cost_weight.segment(mercury::num_virtual, mercury::num_act_joint) 
        //= dynacore::Vector::Constant(mercury::num_act_joint, 100000.0);

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 10000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Joint Position Control] Constructed\n");
}

JPosPostureFixCtrl::~JPosPostureFixCtrl(){
    delete jpos_task_;
    delete contact_constraint_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosPostureFixCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(mercury::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    _contact_constraint_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void JPosPostureFixCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
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

void JPosPostureFixCtrl::_jpos_task_setup(){
    des_jvel_.setZero();
    dynacore::Vector qdot_des(mercury::num_qdot);
    qdot_des.setZero();
    dynacore::Vector qddot_des(mercury::num_qdot);
    qddot_des.setZero();

    sp_->curr_jpos_des_[mercury_joint::leftHip - mercury::num_virtual] += 
        sp_->Kp_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
    sp_->curr_jpos_des_[mercury_joint::rightHip - mercury::num_virtual] += 
        sp_->Kp_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];

    des_jacc_[mercury_joint::leftHip - mercury::num_virtual] = 
        sp_->Kd_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
    des_jacc_[mercury_joint::rightHip - mercury::num_virtual] = 
        sp_->Kd_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];

    des_jpos_ = sp_->curr_jpos_des_;
    dynacore::Vector config_des = sp_->Q_;
    config_des.segment(mercury::num_virtual, mercury::num_act_joint) = 
        des_jpos_;
    


    jpos_task_->UpdateTask(&(config_des), qdot_des, qddot_des);
    task_list_.push_back(jpos_task_);
}

void JPosPostureFixCtrl::_contact_constraint_setup(){
    contact_constraint_->UpdateContactSpec();
    contact_list_.push_back(contact_constraint_);
}

void JPosPostureFixCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;

    sp_->curr_jpos_des_[mercury_joint::rightKnee - mercury::num_virtual] = 
        set_jpos_[mercury_joint::rightKnee - mercury::num_virtual];
    sp_->curr_jpos_des_[mercury_joint::leftKnee - mercury::num_virtual] = 
        set_jpos_[mercury_joint::leftKnee - mercury::num_virtual];
}

void JPosPostureFixCtrl::LastVisit(){
}

bool JPosPostureFixCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosPostureFixCtrl::CtrlInitialization(const std::string & setting_file_name){
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
    wbwc_->Kp_ = ((JPosTask*)jpos_task_)->Kp_vec_.tail(mercury::num_act_joint);
    wbwc_->Kd_ = ((JPosTask*)jpos_task_)->Kd_vec_.tail(mercury::num_act_joint);
}
