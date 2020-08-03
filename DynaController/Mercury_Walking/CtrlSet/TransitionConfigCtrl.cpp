#include "TransitionConfigCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContactBounding.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury_Controller/WBWC.hpp>

TransitionConfigCtrl::TransitionConfigCtrl(RobotSystem* robot, 
        int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(mercury::num_act_joint),
    des_jvel_(mercury::num_act_joint),
    des_jacc_(mercury::num_act_joint)
{
    des_jacc_.setZero();

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 1000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;


    config_task_ = new ConfigTask();
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
                config_task_->getDim() + 
                double_contact_->getDim(), 100.0);

    // wbdc_rotor_data_->cost_weight[0] = 200;    
    // wbdc_rotor_data_->cost_weight[1] = 200;    
    // wbdc_rotor_data_->cost_weight[2] = 200;    

    wbdc_rotor_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[config_task_->getDim() + 2]  = 0.001; // Fr_z
    wbdc_rotor_data_->cost_weight[config_task_->getDim() + 5]  = 0.001; // Fr_z

    sp_ = Mercury_StateProvider::getStateProvider();
    // printf("[Transition Controller] Constructed\n");
}

TransitionConfigCtrl::~TransitionConfigCtrl(){
    delete config_task_;
    delete double_contact_;

    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void TransitionConfigCtrl::OneStep(void* _cmd){
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
    // sp_->curr_jpos_des_ = des_jpos_;
    _PostProcessing_Command();
}

void TransitionConfigCtrl::_body_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    
   // dynacore::Vector fb_cmd = dynacore::Vector::Zero(mercury::num_act_joint);
   //  for (int i(0); i<mercury::num_act_joint; ++i){
   //      wbdc_rotor_data_->A_rotor(i + mercury::num_virtual, i + mercury::num_virtual)
   //          = sp_->rotor_inertia_[i];
   //  }
    
   //  wbdc_rotor_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
   //  wbdc_rotor_->MakeTorque(task_list_, contact_list_, fb_cmd, wbdc_rotor_data_);

   //  gamma = wbdc_rotor_data_->cmd_ff;
    
   //  dynacore::Vector reaction_force = 
   //      (wbdc_rotor_data_->opt_result_).tail(double_contact_->getDim());
   //  for(int i(0); i<double_contact_->getDim(); ++i)
   //      sp_->reaction_forces_[i] = reaction_force[i];

   //  sp_->qddot_cmd_ = wbdc_rotor_data_->result_qddot_;
   //  sp_->reflected_reaction_force_ = wbdc_rotor_data_->reflected_reaction_force_;

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

void TransitionConfigCtrl::_body_task_setup(){
    dynacore::Vector pos_des(config_task_->getDim()); // not exact orientation task
    dynacore::Vector vel_des(config_task_->getDim());
    dynacore::Vector acc_des(config_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // Body Height
    double target_height = ini_body_pos_[2];
    if(b_set_height_target_) target_height = des_body_height_;

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    // TEST
    rpy_des[1] = sp_->des_body_pitch_;

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();

    dynacore::Vector config_sol;
    inv_kin_.getDoubleSupportLegConfig(sp_->Q_, quat_des, 
        target_height, config_sol);
    for (int i(0); i<mercury::num_act_joint; ++i){       
        des_jpos_[i] = config_sol[mercury::num_virtual + i];
        des_jvel_[i] = 0.;
    }

    // TEST
    if(b_increase_){
        if(moving_foot_ == mercury_link::rightFoot){
            int swing_jidx = mercury_joint::rightAbduction - mercury::num_virtual;

            double h(state_machine_time_/end_time_);
            
            for(int i(0); i<3; ++i){
                des_jpos_[i + swing_jidx] = 
                    h * config_sol[mercury_joint::rightAbduction + i]
                     + (1. - h)*sp_->curr_jpos_des_[swing_jidx + i];
             }
        }
        else if(moving_foot_ == mercury_link::leftFoot){
            int swing_jidx = mercury_joint::leftAbduction - mercury::num_virtual;

            double h(state_machine_time_/end_time_);
            
            for(int i(0); i<3; ++i){
                des_jpos_[i + swing_jidx] = 
                    h * config_sol[mercury_joint::leftAbduction + i]
                     + (1. - h)*sp_->curr_jpos_des_[swing_jidx + i];
             }
        }
    }
}

void TransitionConfigCtrl::_double_contact_setup(){
  double alpha = 0.5 * (1-cos(M_PI * state_machine_time_/end_time_));
   double upper_lim(100.);
   double rf_weight(100.);
   double rf_weight_z(100.);
   double foot_weight(1000.);

    if(b_increase_){
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_));

        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha)  + 1000. * alpha;
    } else {
        ((DoubleContactBounding*)double_contact_)->setFzUpperLimit(
            max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_));

        upper_lim = max_rf_z_ - alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (alpha) * 5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha) * 0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha)  + 1000. * (1. - alpha);
     }
    //((DoubleContactBounding*)double_contact_)->setFrictionCoeff(0.3*(1-alpha), 0.3);


    double_contact_->UpdateContactSpec();
    contact_list_.push_back(double_contact_);

    if(moving_foot_ == mercury_link::leftFoot) {
        wbwc_->W_rf_[3] = rf_weight;
        wbwc_->W_rf_[4] = rf_weight;
        wbwc_->W_rf_[5] = rf_weight_z;

        wbwc_->W_foot_[3] = foot_weight;
        wbwc_->W_foot_[4] = foot_weight;
        wbwc_->W_foot_[5] = foot_weight;

        wbwc_->left_z_max_ = upper_lim; 
    }
    else if(moving_foot_ == mercury_link::rightFoot) {
        wbwc_->W_rf_[0] = rf_weight;
        wbwc_->W_rf_[1] = rf_weight;
        wbwc_->W_rf_[2] = rf_weight_z;

        wbwc_->W_foot_[0] = foot_weight;
        wbwc_->W_foot_[1] = foot_weight;
        wbwc_->W_foot_[2] = foot_weight;

        wbwc_->right_z_max_ = upper_lim; 
    }
}

void TransitionConfigCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ctrl_start_time_ = sp_->curr_time_;
}

void TransitionConfigCtrl::LastVisit(){
    sp_->curr_jpos_des_ = des_jpos_;
    // printf("[Transition] End\n");
}

bool TransitionConfigCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void TransitionConfigCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_body_pos_);
    std::vector<double> tmp_vec;
    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_task_)->Kp_vec_[i] = tmp_vec[i];
    }

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((ConfigTask*)config_task_)->Kd_vec_[i] = tmp_vec[i];
    }
    wbwc_->Kp_ = ((ConfigTask*)config_task_)->Kp_vec_.tail(mercury::num_act_joint);
    wbwc_->Kd_ = ((ConfigTask*)config_task_)->Kd_vec_.tail(mercury::num_act_joint);
}
