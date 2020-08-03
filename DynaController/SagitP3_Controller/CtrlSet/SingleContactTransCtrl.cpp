#include "SingleContactTransCtrl.hpp"
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <SagitP3_Controller/TaskSet/BodyTask.hpp>

#include <SagitP3_Controller/ContactSet/SingleContact.hpp>
#include <SagitP3_Controller/ContactSet/SingleFullContact.hpp>

#include <WBLC/KinWBC.hpp>
#include <WBLC/WBLC.hpp>
#include <SagitP3/SagitP3_Model.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

SingleContactTransCtrl::SingleContactTransCtrl(const RobotSystem* robot, 
        int moving_foot, bool b_increase):
    Controller(robot),
    b_set_height_target_(false),
    moving_foot_(moving_foot),
    b_increase_(b_increase),
    end_time_(100.),
    ctrl_start_time_(0.),
    des_jpos_(sagitP3::num_act_joint),
    des_jvel_(sagitP3::num_act_joint),
    des_jacc_(sagitP3::num_act_joint),
    Kp_(sagitP3::num_act_joint),
    Kd_(sagitP3::num_act_joint)
{
    rfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::r_foot);
    lfoot_contact_ = new SingleFullContact(robot_sys_, sagitP3_link::l_foot);
    dim_contact_ = rfoot_contact_->getDim() + lfoot_contact_->getDim();

    std::vector<bool> act_list;
    act_list.resize(sagitP3::num_qdot, true);
    for(int i(0); i<sagitP3::num_virtual; ++i) act_list[i] = false;

    kin_wbc_ = new KinWBC(act_list);
    wblc_ = new WBLC(act_list);
    wblc_data_ = new WBLC_ExtraData();
    wblc_data_->W_qddot_ = dynacore::Vector::Constant(sagitP3::num_qdot, 100.0);
    wblc_data_->W_rf_ = dynacore::Vector::Constant(dim_contact_, 1.0);
    wblc_data_->W_xddot_ = dynacore::Vector::Constant(dim_contact_, 1000.0);
    wblc_data_->W_rf_[rfoot_contact_->getDim()-1] = 0.01;
    wblc_data_->W_rf_[dim_contact_-1] = 0.01;

    // torque limit default setting
    wblc_data_->tau_min_ = dynacore::Vector::Constant(sagitP3::num_act_joint, -100.);
    wblc_data_->tau_max_ = dynacore::Vector::Constant(sagitP3::num_act_joint, 100.);

    base_task_ = new BodyTask(robot_sys_);
    sp_ = SagitP3_StateProvider::getStateProvider();
    printf("[Transition Controller] Constructed\n");
}

SingleContactTransCtrl::~SingleContactTransCtrl(){
    delete base_task_;
    delete rfoot_contact_;
    delete lfoot_contact_;

    delete kin_wbc_;
    delete wblc_;
    delete wblc_data_;
}

void SingleContactTransCtrl::OneStep(void* _cmd){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    dynacore::Vector gamma;

    _contact_setup();
    _task_setup();
    _compute_torque_wblc(gamma);
    
    for(int i(0); i<sagitP3::num_act_joint; ++i){
        ((SagitP3_Command*)_cmd)->jtorque_cmd[i] = gamma[i];
        ((SagitP3_Command*)_cmd)->jpos_cmd[i] = des_jpos_[i];
        ((SagitP3_Command*)_cmd)->jvel_cmd[i] = des_jvel_[i];
    }
    _PostProcessing_Command();
}

void SingleContactTransCtrl::_compute_torque_wblc(dynacore::Vector & gamma){
    dynacore::Matrix A_rotor = A_;
    for (int i(0); i<sagitP3::num_act_joint; ++i){
        A_rotor(i + sagitP3::num_virtual, i + sagitP3::num_virtual)
            += sp_->rotor_inertia_[i];
    }
    dynacore::Matrix A_rotor_inv = A_rotor.inverse();

    wblc_->UpdateSetting(A_rotor, A_rotor_inv, coriolis_, grav_);
    dynacore::Vector des_jacc_cmd = des_jacc_ 
        + Kp_.cwiseProduct(des_jpos_ - 
                sp_->Q_.segment(sagitP3::num_virtual, sagitP3::num_act_joint))
        + Kd_.cwiseProduct(des_jvel_ - sp_->Qdot_.tail(sagitP3::num_act_joint));

    wblc_->MakeWBLC_Torque(
            des_jacc_cmd, contact_list_,
            gamma, wblc_data_);

    //dynacore::pretty_print(des_jpos_, std::cout, "des_jpos");
    //dynacore::pretty_print(des_jvel_, std::cout, "des_jvel");
    //dynacore::pretty_print(des_jacc_, std::cout, "des_jacc");
    //dynacore::pretty_print(des_jacc_cmd, std::cout, "des_jacc");
    //dynacore::pretty_print(gamma, std::cout, "gamma");

    sp_->qddot_cmd_ = wblc_data_->qddot_;
    sp_->reaction_forces_ = wblc_data_->Fr_;
}

void SingleContactTransCtrl::_task_setup(){
    // Body height
    double base_height_cmd = ini_base_height_;
    if(b_set_height_target_) base_height_cmd = des_base_height_;
 
    // Orientation
    dynacore::Quaternion des_quat;
    dynacore::convert(0., 0., M_PI/2., des_quat);
   
    dynacore::Vector pos_des(7); pos_des.setZero();
    dynacore::Vector vel_des(base_task_->getDim()); vel_des.setZero();
    dynacore::Vector acc_des(base_task_->getDim()); acc_des.setZero();

    pos_des[0] = des_quat.x();
    pos_des[1] = des_quat.y();
    pos_des[2] = des_quat.z();
    pos_des[3] = des_quat.w();

    pos_des[4] = ini_base_pos_[0];
    pos_des[5] = ini_base_pos_[1];
    pos_des[6] = base_height_cmd;


    base_task_->UpdateTask(&(pos_des), vel_des, acc_des);
    task_list_.push_back(base_task_);

    kin_wbc_->FindConfiguration(sp_->Q_, task_list_, contact_list_, 
            des_jpos_, des_jvel_, des_jacc_);

    // TEST
    if(b_increase_){
        if(moving_foot_ == sagitP3_link::r_foot){
            int swing_jidx = sagitP3_joint::Abduction_Right - sagitP3::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < sagitP3::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h; 
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
             }
        }
        else if(moving_foot_ == sagitP3_link::l_foot){
            int swing_jidx = sagitP3_joint::Abduction_Left - sagitP3::num_virtual;
            double h(state_machine_time_/end_time_);

            for(int i(0); i < sagitP3::num_leg_joint; ++i){
                des_jpos_[i + swing_jidx] *= h;
                des_jpos_[i + swing_jidx] += 
                    (1. - h)*sp_->des_jpos_prev_[swing_jidx + i];
            }
        }
    }
}

void SingleContactTransCtrl::_contact_setup(){
    double alpha = 0.5 * (1-cos(M_PI * state_machine_time_/end_time_));
    double upper_lim(100.);
    double rf_weight(100.);
    double rf_weight_z(100.);
    double foot_weight(1000.);

    if(b_increase_){
        upper_lim = min_rf_z_ + alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (1. - alpha) * 5. + alpha * 1.0;
        rf_weight_z = (1. - alpha) * 0.5 + alpha * 0.01;
        foot_weight = 0.001 * (1. - alpha)  + 1000. * alpha;
    } else {
        upper_lim = max_rf_z_ - alpha*(max_rf_z_ - min_rf_z_);
        rf_weight = (alpha) * 5. + (1. - alpha) * 1.0;
        rf_weight_z = (alpha) * 0.5 + (1. - alpha) * 0.01;
        foot_weight = 0.001 * (alpha)  + 1000. * (1. - alpha);
    }
    rfoot_contact_->UpdateContactSpec();
    lfoot_contact_->UpdateContactSpec();

    contact_list_.push_back(rfoot_contact_);
    contact_list_.push_back(lfoot_contact_);

    int idx_offset(0);
    if(moving_foot_ == sagitP3_link::l_foot) {
        idx_offset = rfoot_contact_->getDim();
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + idx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + idx_offset] = foot_weight;
        }

        wblc_data_->W_rf_[lfoot_contact_->getDim() -1 + idx_offset] = rf_weight_z;
        ((SingleContact*)lfoot_contact_)->setMaxFz(upper_lim); 
    }
    else if(moving_foot_ == sagitP3_link::r_foot) {
        for(int i(0); i<lfoot_contact_->getDim(); ++i){
            wblc_data_->W_rf_[i + idx_offset] = rf_weight;
            wblc_data_->W_xddot_[i + idx_offset] = foot_weight;
        }
        wblc_data_->W_rf_[rfoot_contact_->getDim() -1 + idx_offset] = rf_weight_z;

        ((SingleContact*)rfoot_contact_)->setMaxFz(upper_lim); 
    }
}

void SingleContactTransCtrl::FirstVisit(){
    // printf("[Transition] Start\n");
    ini_base_height_ = sp_->Q_[sagitP3_joint::virtual_Z];
    ctrl_start_time_ = sp_->curr_time_;

    robot_sys_->getPos(sagitP3_link::hip_ground, ini_base_pos_);
}

void SingleContactTransCtrl::LastVisit(){
    sp_->des_jpos_prev_ = des_jpos_;
    // printf("[Transition] End\n");
}

bool SingleContactTransCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void SingleContactTransCtrl::CtrlInitialization(const std::string & setting_file_name){
    std::vector<double> tmp_vec;
    ParamHandler handler(SagitP3ConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kp_[i] = tmp_vec[i];

    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i) Kd_[i] = tmp_vec[i];
}
