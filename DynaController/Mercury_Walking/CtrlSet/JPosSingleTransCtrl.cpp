#include "JPosSingleTransCtrl.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/JPosTask.hpp>
#include <Mercury_Controller/ContactSet/FixedBodyContact.hpp>

#include <Mercury_Controller/TaskSet/ConfigTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleContactBounding.hpp>

#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury_Controller/WBWC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

JPosSingleTransCtrl::JPosSingleTransCtrl(RobotSystem* robot,
        int moving_foot, bool b_increase):
    Controller(robot),
    swing_foot_(moving_foot),
    b_increase_(b_increase),
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
    //contact_ = new FixedBodyContact(robot);
    jpos_task_ = new ConfigTask();
    contact_ = new DoubleContactBounding(robot, moving_foot);

    wbwc_ = new WBWC(robot);
    wbwc_->W_virtual_ = dynacore::Vector::Constant(6, 100.0);
    wbwc_->W_rf_ = dynacore::Vector::Constant(6, 1.0);
    wbwc_->W_foot_ = dynacore::Vector::Constant(6, 10000.0);
    wbwc_->W_rf_[2] = 0.01;
    wbwc_->W_rf_[5] = 0.01;


    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);
    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(contact_->getDim() + 
                jpos_task_->getDim(), 1000.0);

    wbdc_rotor_data_->cost_weight.tail(contact_->getDim()) = 
        dynacore::Vector::Constant(contact_->getDim(), 0.1);

    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 2] = 0.001;
    wbdc_rotor_data_->cost_weight[jpos_task_->getDim() + 5] = 0.001;

    //wbdc_rotor_data_->cost_weight.head(mercury::num_virtual) 
        //= dynacore::Vector::Constant(mercury::num_virtual, 0.001);
    //wbdc_rotor_data_->cost_weight.segment(mercury::num_virtual, mercury::num_act_joint) 
        //= dynacore::Vector::Constant(mercury::num_act_joint, 100000.0);
 
    sp_ = Mercury_StateProvider::getStateProvider();

    //printf("[Joint Position Single Transition Controller] Constructeh\n");
}

JPosSingleTransCtrl::~JPosSingleTransCtrl(){
    delete jpos_task_;
    delete contact_;
    delete wbdc_rotor_;
    delete wbdc_rotor_data_;
}

void JPosSingleTransCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    dynacore::Vector gamma = dynacore::Vector::Zero(mercury::num_act_joint);
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    _contact_setup();
    _jpos_task_setup();
    _jpos_ctrl_wbdc_rotor(gamma);

    for(int i(0); i<mercury::num_act_joint; ++i){
        ((Mercury_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((Mercury_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((Mercury_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }

    _PostProcessing_Command();
}

void JPosSingleTransCtrl::_jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma){

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

void JPosSingleTransCtrl::_jpos_task_setup(){
    des_jvel_.setZero();
    dynacore::Vector qdot_des(mercury::num_qdot);
    qdot_des.setZero();
    dynacore::Vector qddot_des(mercury::num_qdot);
    qddot_des.setZero();

    // Abduction roll
    sp_->curr_jpos_des_[stance_leg_jidx_] += 
        sp_->Kp_roll_ * sp_->Q_[mercury_joint::virtual_Rx];
    des_jacc_[stance_leg_jidx_] = 
        sp_->Kd_roll_ * sp_->Q_[mercury_joint::virtual_Rx];
     // Hip Pitch
    sp_->curr_jpos_des_[stance_leg_jidx_ + 1] += 
        sp_->Kp_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
    des_jacc_[stance_leg_jidx_ + 1] = 
        sp_->Kd_pitch_ * sp_->Q_[mercury_joint::virtual_Ry];
 
    int rknee_idx(mercury_joint::rightKnee - mercury::num_virtual);
    int lknee_idx(mercury_joint::leftKnee - mercury::num_virtual);

    sp_->curr_jpos_des_[rknee_idx] =
        dynacore::smooth_changing(initial_rknee_jpos_des_, 
                set_jpos_[rknee_idx], end_time_, state_machine_time_);

    sp_->curr_jpos_des_[lknee_idx] =
        dynacore::smooth_changing(initial_lknee_jpos_des_, 
                set_jpos_[lknee_idx], end_time_, state_machine_time_);

    des_jpos_ = sp_->curr_jpos_des_;

    dynacore::Vector config_des = sp_->Q_;
    config_des.segment(mercury::num_virtual, mercury::num_act_joint) = 
        des_jpos_;
    

    jpos_task_->UpdateTask(&(config_des), qdot_des, qddot_des);
    task_list_.push_back(jpos_task_);
}

void JPosSingleTransCtrl::_contact_setup(){
    //double alpha(state_machine_time_/end_time_);
    // 0 --> 1
    double alpha = 0.5 * (1-cos(M_PI * state_machine_time_/end_time_));
   double upper_lim(100.);
   double rf_weight(100.);
   double rf_weight_z(100.);
   double foot_weight(1000.);

    if(b_increase_){
        ((DoubleContactBounding*)contact_)->setFzUpperLimit(
            min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_));

        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha)  + 10000. * alpha;
    } else {
        ((DoubleContactBounding*)contact_)->setFzUpperLimit(
            max_rf_z_ - alpha * (max_rf_z_ - min_rf_z_));

        upper_lim = max_rf_z_ - alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (alpha) * 5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha) * 0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha)  + 10000. * (1. - alpha);
     }
    //((DoubleContactBounding*)contact_)->setFrictionCoeff(0.3*(1-alpha), 0.3);

    contact_->UpdateContactSpec();
    contact_list_.push_back(contact_);

    if(swing_foot_ == mercury_link::leftFoot) {
        wbwc_->W_rf_[3] = rf_weight;
        wbwc_->W_rf_[4] = rf_weight;
        wbwc_->W_rf_[5] = rf_weight_z;

        wbwc_->W_foot_[3] = foot_weight;
        wbwc_->W_foot_[4] = foot_weight;
        wbwc_->W_foot_[5] = foot_weight;

        wbwc_->left_z_max_ = upper_lim; 
        stance_leg_jidx_ = mercury_joint::rightAbduction - mercury::num_virtual;
    }
    else if(swing_foot_ == mercury_link::rightFoot) {
        wbwc_->W_rf_[0] = rf_weight;
        wbwc_->W_rf_[1] = rf_weight;
        wbwc_->W_rf_[2] = rf_weight_z;

        wbwc_->W_foot_[0] = foot_weight;
        wbwc_->W_foot_[1] = foot_weight;
        wbwc_->W_foot_[2] = foot_weight;

        wbwc_->right_z_max_ = upper_lim; 
        stance_leg_jidx_ = mercury_joint::leftAbduction - mercury::num_virtual;
    }

    //static int count(0);
    //++count;
    //if( count> 100 ){ exit(0); }
}

void JPosSingleTransCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;

    // Set Up desired knee joint position
    initial_rknee_jpos_des_ = 
        sp_->curr_jpos_des_[mercury_joint::rightKnee - mercury::num_virtual];
    initial_lknee_jpos_des_ = 
        sp_->curr_jpos_des_[mercury_joint::leftKnee - mercury::num_virtual];
}

void JPosSingleTransCtrl::LastVisit(){
}

bool JPosSingleTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void JPosSingleTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(mercury::num_virtual, mercury::num_act_joint);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);


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
