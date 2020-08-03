#include "JPosCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>

JPosCtrl::JPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    b_jpos_set_(false),
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

    wbdc_ = new WBDC_Relax(act_list);
    wbdc_data_ = new WBDC_Relax_ExtraData();

    wbdc_data_->tau_min = dynacore::Vector(mercury::num_act_joint);
    wbdc_data_->tau_max = dynacore::Vector(mercury::num_act_joint);
    for(int i(0); i<mercury::num_act_joint; ++i){
        wbdc_data_->tau_max[i] = 50.0;
        wbdc_data_->tau_min[i] = -50.0;
    }
    
    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim() + 
                jpos_task_->getDim(), 100.0);
    wbdc_rotor_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 0.001);

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Joint Position Control] Constructed\n");
}

JPosCtrl::~JPosCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_;
    delete wbdc_data_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _fixed_body_contact_setup();
    _jpos_task_setup();
    //_jpos_ctrl(gamma);
    _jpos_ctrl_wbdc_rotor(gamma);

    _PostProcessing_Command();
}

void JPosCtrl::_jpos_ctrl(dynacore::Vector & gamma){
    dynacore::Vector jtorque_cmd(mercury::num_act_joint);

    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, jtorque_cmd, wbdc_data_);

    gamma.head(mercury::num_act_joint) = jtorque_cmd;

    dynacore::Matrix A_rotor = A_;
    for(int i(0); i<mercury::num_act_joint; ++i) {
        A_rotor(i+mercury::num_virtual,i + mercury::num_virtual) 
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv;

    dynacore::pseudoInverse(A_rotor, 0.00001, A_rotor_inv, 0);
    wbdc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, jtorque_cmd, wbdc_data_);
    gamma.tail(mercury::num_act_joint) = jtorque_cmd;
}

void JPosCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
    for (int i(0); i<mercury::num_act_joint; ++i){
        wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
            = sp_->rotor_inertia_[i];
    }
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma.head(mercury::num_act_joint) = fb_cmd;
    gamma.tail(mercury::num_act_joint) = wbdc_rotor_data_->cmd_ff;

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
                sp_->jpos_des_[i] = set_jpos_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);
            else
                sp_->jpos_des_[i] = jpos_ini_[i] + amp_[i] * sin(omega * state_machine_time_ + phase_[i]);

            sp_->jvel_des_[i] = amp_[i] * omega * cos(omega * state_machine_time_ + phase_[i]);
            sp_->jvel_des_[i] *= ramp_value;
            jacc_des[i] = -amp_[i] * omega * omega * sin(omega * state_machine_time_ + phase_[i]);
            jacc_des[i] *= ramp_value;
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
                sp_->jpos_des_[i] = ini_jpos;
            }else if(state_machine_time_<start_time_[i] + delta_time_[i]){
                sp_->jpos_des_[i] = ini_jpos + jpos_delta_[i]*rising_time/delta_time_[i];
            }else {
                sp_->jpos_des_[i] = ini_jpos + jpos_delta_[i];
            }
            sp_->jvel_des_[i] = 0.;
        }
    }
    jpos_task_->UpdateTask(&(sp_->jpos_des_), sp_->jvel_des_, jacc_des);

    std::vector<bool> relaxed_op(jpos_task_->getDim(), false);
    jpos_task_->setRelaxedOpCtrl(relaxed_op);
    task_list_.push_back(jpos_task_);
}

void JPosCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
    wbdc_data_->cost_weight = dynacore::Vector::Zero(fixed_body_contact_->getDim());

    for(int i(0); i<fixed_body_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[i] = 1.;
    }
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
