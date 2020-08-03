#include "ContactTransConfigCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleTransitionContact.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

ContactTransConfigCtrl::ContactTransConfigCtrl(RobotSystem* robot):
    Controller(robot),
    b_set_height_target_(false),
    end_time_(100.),
    body_pos_ini_(4),
	des_jpos_(mercury::num_act_joint),
	des_jvel_(mercury::num_act_joint),
    config_des_(mercury::num_qdot)
{
    config_des_.setZero();
    body_task_ = new ConfigTask();
    double_contact_ = new DoubleTransitionContact(robot);

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

    //wbdc_rotor_data_->cost_weight[0] = 0.0001; // X
    //wbdc_rotor_data_->cost_weight[1] = 0.0001; // Y
    //wbdc_rotor_data_->cost_weight[5] = 0.0001; // Yaw
    
    wbdc_rotor_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[body_task_->getDim() + 2]  = 0.001; // Fr_z
    wbdc_rotor_data_->cost_weight[body_task_->getDim() + 5]  = 0.001; // Fr_z

   sp_ = Mercury_StateProvider::getStateProvider();

   inv_kin_ = new Mercury_InvKinematics();

    printf("[Contact Transition Body Ctrl] Constructed\n");
}

ContactTransConfigCtrl::~ContactTransConfigCtrl(){
    delete body_task_;
    delete double_contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;

    delete inv_kin_;
}

void ContactTransConfigCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
	state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

	dynacore::Vector gamma;
	_double_contact_setup();
	_body_task_setup();
	_body_ctrl_wbdc_rotor(gamma);
	for(int i(0); i<mercury::num_act_joint; ++i){
		((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
		((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
		((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
	}
    _PostProcessing_Command();
}

void ContactTransConfigCtrl::_body_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    
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

    double ramp = (state_machine_time_)/(end_time_*0.5);
    if( state_machine_time_ > end_time_* 0.5 ) ramp = 1.;
    dynacore::Vector ramp_grav = ramp * grav_;
     
    //wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, ramp_grav);
    wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

    gamma = wbdc_rotor_data_->cmd_ff;

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


void ContactTransConfigCtrl::_body_task_setup(){
    double body_height_cmd;
    dynacore::Vector vel_des(body_task_->getDim());
    dynacore::Vector acc_des(body_task_->getDim());
    vel_des.setZero(); acc_des.setZero();

    // Body Pos
    if(b_set_height_target_){
        body_height_cmd = dynacore::smooth_changing(ini_body_pos_[2], des_body_height_, end_time_, state_machine_time_);
    }else{
        printf("[Warning] The body height is not specified\n");
    }
    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion des_quat;
    rpy_des.setZero();

    // TEST
    rpy_des[1] = sp_->des_body_pitch_;

    dynacore::convert(rpy_des, des_quat);
    dynacore::Vector config_sol; 

    inv_kin_->getDoubleSupportLegConfig(sp_->Q_, des_quat, body_height_cmd, config_sol);
    sp_->body_pos_des_[2] = body_height_cmd;
    sp_->body_ori_des_ = des_quat;

    for (int i(0); i<mercury::num_act_joint; ++i){
        des_jpos_[i] = config_sol[mercury::num_virtual + i];
        config_des_[mercury::num_virtual + i] = des_jpos_[i];
        des_jvel_[i] = 0.;
    }
    body_task_->UpdateTask(&(config_des_), vel_des, acc_des);

   // Push back to task list
    task_list_.push_back(body_task_);

}

void ContactTransConfigCtrl::_double_contact_setup(){
    ((DoubleTransitionContact*)double_contact_)->
        setFzUpperLimit(min_rf_z_ + 
                state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);
}

void ContactTransConfigCtrl::FirstVisit(){
    ini_body_pos_ = sp_->Q_.head(3);
    ctrl_start_time_ = sp_->curr_time_;
}

void ContactTransConfigCtrl::LastVisit(){
    // printf("[ContactTransBody] End\n");
}

bool ContactTransConfigCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void ContactTransConfigCtrl::CtrlInitialization(const std::string & setting_file_name){
    ini_body_pos_ = sp_->Q_.head(3);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);
    // Feedback Gain
    std::vector<double> tmp_vec;
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)body_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)body_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
