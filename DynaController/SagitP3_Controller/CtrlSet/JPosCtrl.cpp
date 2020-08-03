#include "JPosCtrl.hpp"
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <SagitP3_Controller/TaskSet/JPosTask.hpp>
#include <SagitP3_Controller/ContactSet/FixedBodyContact.hpp>
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <WBDC/WBDC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
    des_jpos_(sagitP3::num_act_joint),
    des_jvel_(sagitP3::num_act_joint),
   ctrl_start_time_(0.)
{
    set_jpos_.resize(sagitP3::num_act_joint, 0.);
    amp_.resize(sagitP3::num_act_joint, 0.);
    freq_.resize(sagitP3::num_act_joint, 0.);
    phase_.resize(sagitP3::num_act_joint, 0.);

    jpos_task_ = new JPosTask();
    fixed_body_contact_ = new FixedBodyContact(robot);
    std::vector<bool> act_list;
    act_list.resize(sagitP3::num_qdot, true);
    for(int i(0); i<sagitP3::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC(act_list);
    wbdc_data_ = new WBDC_ExtraData();
    wbdc_data_->cost_weight = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);
    wbdc_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 0.1);

    sp_ = SagitP3_StateProvider::getStateProvider();

    printf("[ CTRL - Joint Position] Constructed\n");
}

JPosCtrl::~JPosCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_;
    delete wbdc_data_;
}

void JPosCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(sagitP3::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    _fixed_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<sagitP3::num_act_joint; ++i){
        ((SagitP3_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((SagitP3_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((SagitP3_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void JPosCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
    //dynacore::pretty_print(grav_, std::cout, "gravity");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
}

void JPosCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(sagitP3::num_act_joint); jacc_des.setZero();

    double ramp_value(0.);
    double ramp_duration(0.5);

    if(state_machine_time_ < ramp_duration){
        ramp_value = state_machine_time_/ramp_duration;
    }else{
        ramp_value = 1.;
    }

    double omega;

    for(int i(0); i<sagitP3::num_act_joint; ++i){
        omega = 2. * M_PI * freq_[i];
        if(b_jpos_set_)
            des_jpos_[i] = set_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
        else
            des_jpos_[i] = jpos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);

        des_jvel_[i] = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
        des_jvel_[i] *= ramp_value;
        jacc_des[i] = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
        jacc_des[i] *= ramp_value;
    }
    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, jacc_des);
    task_list_.push_back(jpos_task_);
}

void JPosCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
}

void JPosCtrl::LastVisit(){  }

bool JPosCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint);

    ParamHandler handler(SagitP3ConfigPath + setting_file_name + ".yaml");

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
}
