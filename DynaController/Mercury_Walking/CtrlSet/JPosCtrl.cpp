#include "JPosCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
   ctrl_start_time_(0.)
{
    set_jpos_.resize(mercury::num_act_joint, 0.);
    amp_.resize(mercury::num_act_joint, 0.);
    freq_.resize(mercury::num_act_joint, 0.);
    phase_.resize(mercury::num_act_joint, 0.);

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
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 0.1);

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Joint Position Control] Constructed\n");
}

JPosCtrl::~JPosCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(mercury::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

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

void JPosCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
}

void JPosCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(mercury::num_act_joint); jacc_des.setZero();

    double ramp_value(0.);
    double ramp_duration(0.5);

    if(state_machine_time_ < ramp_duration){
        ramp_value = state_machine_time_/ramp_duration;
    }else{
        ramp_value = 1.;
    }

   if(trj_type_ == mercury_trj_type::sinusoidal){
        double omega;

        for(int i(0); i<mercury::num_act_joint; ++i){
            omega = 2. * M_PI * freq_[i];
            if(b_jpos_set_)
                des_jpos_[i] = set_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
            else
                des_jpos_[i] = jpos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);

            des_jvel_[i] = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
            des_jvel_[i] *= ramp_value;
            jacc_des[i] = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
            jacc_des[i] *= ramp_value;
            // TEST
            //jacc_des[i] *= 0.;
            //jacc_des[i] += des_jvel_[i];
        }
    } else if(trj_type_ == mercury_trj_type::ramp){
        double rising_time;
        double ini_jpos;

        for (int i(0); i<mercury::num_act_joint; ++i){
            rising_time = state_machine_time_ - start_time_[i];
            if(b_jpos_set_){
                ini_jpos = set_jpos_[i];
            }else {
                ini_jpos = jpos_ini_[i];
            }
            if(state_machine_time_ < start_time_[i]){
                des_jpos_[i] = ini_jpos;
            }else if(state_machine_time_<start_time_[i] + delta_time_[i]){
                des_jpos_[i] = ini_jpos + jpos_delta_[i]*rising_time/delta_time_[i];
            }else {
                des_jpos_[i] = ini_jpos + jpos_delta_[i];
            }
            des_jvel_[i] = 0.;
        }
    }
    jpos_task_->UpdateTask(&(des_jpos_), des_jvel_, jacc_des);
    task_list_.push_back(jpos_task_);
}

void JPosCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
}

void JPosCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
}

void JPosCtrl::LastVisit(){
}

bool JPosCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosCtrl::CtrlInitialization(const std::string & setting_file_name){
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
}
