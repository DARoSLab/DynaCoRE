#include "JPosTargetCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/utilities.hpp>

JPosTargetCtrl::JPosTargetCtrl(RobotSystem* robot):Controller(robot),
    jpos_target_(mercury::num_act_joint),
    end_time_(1000.0),
    ctrl_start_time_(0.)
{
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
                jpos_task_->getDim(), 1000.0);

    wbdc_rotor_data_->cost_weight.tail(fixed_body_contact_->getDim()) = 
        dynacore::Vector::Constant(fixed_body_contact_->getDim(), 1.0);

    sp_ = Mercury_StateProvider::getStateProvider();
}

JPosTargetCtrl::~JPosTargetCtrl(){
    delete jpos_task_;
    delete fixed_body_contact_;
    delete wbdc_;
    delete wbdc_data_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosTargetCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma.setZero();
    _fixed_body_contact_setup();
    _jpos_task_setup();
    //_jpos_ctrl(gamma);
    _jpos_ctrl_wbdc_rotor(gamma);
    //dynacore::pretty_print(gamma, std::cout, "gamma");

    _PostProcessing_Command();
}

void JPosTargetCtrl::_jpos_ctrl(dynacore::Vector & gamma){
    wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);
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

    gamma.head(mercury::num_act_joint) = fb_cmd;
    gamma.tail(mercury::num_act_joint) = wbdc_rotor_data_->cmd_ff;


    sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
}


void JPosTargetCtrl::_jpos_task_setup(){
    dynacore::Vector jacc_des(mercury::num_act_joint); jacc_des.setZero();

    for(int i(0); i<mercury::num_act_joint; ++i){
        sp_->jpos_des_[i] = dynacore::smooth_changing(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        sp_->jvel_des_[i] = dynacore::smooth_changing_vel(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);
        jacc_des[i] = dynacore::smooth_changing_acc(jpos_ini_[i], jpos_target_[i], end_time_, state_machine_time_);

    }
    jpos_task_->UpdateTask(&(sp_->jpos_des_), sp_->jvel_des_, jacc_des);

    std::vector<bool> relaxed_op(jpos_task_->getDim(), false);
    // All relax
    //std::vector<bool> relaxed_op(jpos_task_->getDim(), true);
    //int prev_size(wbdc_data_->cost_weight.rows());
    //wbdc_data_->cost_weight.conservativeResize( prev_size + jpos_task_->getDim());
    //for (int i(0); i<jpos_task_->getDim(); ++i){
        //wbdc_data_->cost_weight[prev_size + i] = 1000.0;
    //}

    jpos_task_->setRelaxedOpCtrl(relaxed_op);
    task_list_.push_back(jpos_task_);
}

void JPosTargetCtrl::_fixed_body_contact_setup(){
    fixed_body_contact_->UpdateContactSpec();
    contact_list_.push_back(fixed_body_contact_);
    wbdc_data_->cost_weight = dynacore::Vector::Zero(fixed_body_contact_->getDim());

    for(int i(0); i<fixed_body_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[i] = 1.;
    }
}

void JPosTargetCtrl::FirstVisit(){
    ctrl_start_time_ = sp_->curr_time_;
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
