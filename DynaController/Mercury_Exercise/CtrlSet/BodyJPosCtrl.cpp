#include "BodyJPosCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>
#include <Mercury_Controller/ContactSet/DoubleContact.hpp>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Definition.h>

BodyJPosCtrl::BodyJPosCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false)
{
    jpos_task_ = new ConfigTask();
    double_body_contact_ = new DoubleContact(robot);
    
    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;
    
    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor =  dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
   
    wbdc_rotor_data_->cost_weight = 
    dynacore::Vector::Constant(
        jpos_task_->getDim() + double_body_contact_->getDim(), 100.0);

    // for(int i(0); i<mercury::num_virtual; ++i){
    //     wbdc_rotor_data_->cost_weight[i] = 0.00001;
    // }
    wbdc_rotor_data_->cost_weight[0] = 10;    
    wbdc_rotor_data_->cost_weight[1] = 10;    
    wbdc_rotor_data_->cost_weight[2] = 200;    

    wbdc_rotor_data_->cost_weight.tail(double_body_contact_->getDim()) = 
        dynacore::Vector::Constant(double_body_contact_->getDim(), 100);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Joint Position Control] Constructed\n");
}

BodyJPosCtrl::~BodyJPosCtrl(){
    delete jpos_task_;
    delete double_body_contact_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void BodyJPosCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2);
    _double_body_contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    _PostProcessing_Command();
}

void BodyJPosCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){
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
    dynacore::Vector reaction_force = 
             (wbdc_rotor_data_->opt_result_).tail(double_body_contact_->getDim());
    for(int i(0); i<double_body_contact_->getDim(); ++i)
        sp_->reaction_forces_[i] = reaction_force[i];
    sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;
}

void BodyJPosCtrl::_jpos_task_setup(){
    // Calculate IK for a desired height and orientation.
    dynacore::Vector Q_cur = sp_->Q_;
    dynacore::Vector config_sol;

    double body_height_cmd;

    // Set Desired Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();
    dynacore::convert(rpy_des, des_quat);    

    dynacore::Vector jpos_des(mercury::num_qdot); jpos_des.setZero();
    dynacore::Vector jvel_des(mercury::num_qdot); jvel_des.setZero();
    dynacore::Vector jacc_des(mercury::num_qdot); jacc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;
    
    inv_kin_.getDoubleSupportLegConfig(Q_cur, des_quat, body_height_cmd, config_sol);
    for (int i(0); i<mercury::num_act_joint; ++i){
        jpos_des[mercury::num_virtual + i] = config_sol[mercury::num_virtual + i];  
        sp_->jpos_des_[i] = jpos_des[mercury::num_virtual + i];
    }

    //dynacore::pretty_print(Q_cur, std::cout, "Q_cur");
    //dynacore::pretty_print(config_sol, std::cout, "config_sol");
    // Maintain initial joint position desired
    jpos_task_->UpdateTask(&(jpos_des), jvel_des, jacc_des);
    task_list_.push_back(jpos_task_);
}

void BodyJPosCtrl::_double_body_contact_setup(){
    double_body_contact_->UpdateContactSpec();
    contact_list_.push_back(double_body_contact_);
}

void BodyJPosCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;
    ini_body_height_ = sp_->Q_[mercury_joint::virtual_Z];
}

void BodyJPosCtrl::LastVisit(){
}

bool BodyJPosCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyJPosCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)jpos_task_)->Kp_vec_[i] = tmp_vec[i];        

    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)jpos_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
