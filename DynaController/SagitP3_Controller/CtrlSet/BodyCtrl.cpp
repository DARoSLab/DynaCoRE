#include "BodyCtrl.hpp"
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <SagitP3_Controller/TaskSet/BodyTask.hpp>

#include <SagitP3_Controller/ContactSet/SingleContact.hpp>
#include <SagitP3_Controller/ContactSet/SingleFullContact.hpp>

#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>

BodyCtrl::BodyCtrl(RobotSystem* robot):Controller(robot),
    end_time_(1000.0),
    ctrl_start_time_(0.),
    b_set_height_target_(false),
    des_jpos_(sagitP3::num_act_joint),
    des_jvel_(sagitP3::num_act_joint),
    des_jacc_(sagitP3::num_act_joint),
    Kp_(sagitP3::num_act_joint),
    Kd_(sagitP3::num_act_joint)
{
    std::vector<bool> act_list;
    act_list.resize(sagitP3::num_qdot, true);
    for(int i(0); i<sagitP3::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    body_task_ = new BodyTask(robot);
    
    //rfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::l_ankle);
    //lfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::r_ankle);

    rfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::l_foot);
    lfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::r_foot);
    
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    wblc_data_->W_qddot_ = dynacore::Vector::Constant(sagitP3::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 0.1);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getDim()-1] = 0.01;
    wblc_data_->W_rf_[dim_contact_-1] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(sagitP3::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(sagitP3::num_act_joint, 100.);

    sp_ = SagitP3_StateProvider::getStateProvider();

    printf("[Body Control] Constructed\n");
}

BodyCtrl::~BodyCtrl(){
    delete body_task_;
    delete wblc_;
    delete wblc_data_;

    delete rfoot_contact_;
    delete lfoot_contact_;
}

void BodyCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;

    dynacore::Vector gamma = dynacore::Vector::Zero(sagitP3::num_act_joint);
    _double_contact_setup();
    _body_task_setup();
    _compute_torque_wblc(gamma);

    for(int i(0); i<sagitP3::num_act_joint; ++i){
        ((SagitP3_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((SagitP3_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((SagitP3_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void BodyCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    // WBLC
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<sagitP3::num_act_joint; ++i){
        A_rotor(i + sagitP3::num_virtual, i + sagitP3::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();

    std::vector<Task*> dummy_task_list_;

    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(sagitP3::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;

    //printf("Body Ctrl\n");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "jacc cmd");
    //dynacore::pretty_print(gamma, std::cout, "gamma");
    //dynacore::pretty_print(sp_->reaction_forces_, std::cout, "rforce");
}

void BodyCtrl::_body_task_setup(){
    des_jpos_ = jpos_ini_;
    des_jvel_.setZero();
    des_jacc_.setZero();
    // Calculate IK for a desired height and orientation.
    double body_height_cmd;

    // Set Desired Orientation
    dynacore::Quaternion des_quat;
    dynacore::convert(0., 0., M_PI/2., des_quat);    

    dynacore::Vector pos_des(7); pos_des.setZero();
    dynacore::Vector vel_des(6); vel_des.setZero();
    dynacore::Vector acc_des(6); acc_des.setZero();

    if(b_set_height_target_) body_height_cmd = target_body_height_;
    else body_height_cmd = ini_body_height_;

    // Orientation
    pos_des[0] = des_quat.x();
    pos_des[1] = des_quat.y();
    pos_des[2] = des_quat.z();
    pos_des[3] = des_quat.w();
    // Position
    //pos_des.tail(3) = ini_body_pos_;
    pos_des[4] = ini_body_pos_[0];
    pos_des[5] = ini_body_pos_[1];
    pos_des[6] = body_height_cmd;

    double amp(0.0);
    double omega(0.5 * 2. * M_PI);

    pos_des[6] += amp * sin(omega * state_machine_time_);
    vel_des[5] = amp * omega * cos(omega * state_machine_time_);

    body_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(body_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);
    
    //dynacore::pretty_print(jpos_ini_, std::cout, "jpos ini");
    //dynacore::pretty_print(des_jpos_, std::cout, "des jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des jacc");
    //
    //dynacore::Vect3 com_pos;
    //robot_sys_->getCoMPosition(com_pos);
    //dynacore::pretty_print(com_pos, std::cout, "com_pos");
}

void BodyCtrl::_double_contact_setup(){
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);
}

void BodyCtrl::FirstVisit(){
    jpos_ini_ = sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint);
    ctrl_start_time_ = sp_->curr_time_;

    robot_sys_->getPos(sagitP3_link::hip_ground, ini_body_pos_);
}

void BodyCtrl::LastVisit(){  }

bool BodyCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void BodyCtrl::CtrlInitialization(const std::string & setting_file_name){
    jpos_ini_ = sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint);
    ParamHandler handler(SagitP3ConfigPath + setting_file_name + ".yaml");

    std::vector<double> tmp_vec;
    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kp_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        Kd_[i] = tmp_vec[i];
    }
}
