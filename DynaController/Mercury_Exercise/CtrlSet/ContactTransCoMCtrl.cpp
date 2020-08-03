#include "ContactTransCoMCtrl.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury_Controller/TaskSet/CoMOriTask.hpp>
#include <Mercury_Controller/ContactSet/DoubleTransitionContact.hpp>
#include <WBDC_Relax/WBDC_Relax.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <WBDC_Rotor/WBDC_Rotor.hpp>
#include <Mercury/Mercury_Definition.h>
#include <ParamHandler/ParamHandler.hpp>

ContactTransCoMCtrl::ContactTransCoMCtrl(RobotSystem* robot):
    Controller(robot),
    b_set_height_target_(false),
    end_time_(100.),
    com_pos_ini_(4)
{
    com_task_ = new CoMOriTask(robot);
    double_contact_ = new DoubleTransitionContact(robot);

    std::vector<bool> act_list;
    act_list.resize(mercury::num_qdot, true);
    for(int i(0); i<mercury::num_virtual; ++i) act_list[i] = false;

    wbdc_ = new WBDC_Relax(act_list);

    wbdc_data_ = new WBDC_Relax_ExtraData();
    wbdc_data_->tau_min = dynacore::Vector(mercury::num_act_joint);
    wbdc_data_->tau_max = dynacore::Vector(mercury::num_act_joint);

    for(int i(0); i<mercury::num_act_joint; ++i){
        wbdc_data_->tau_max[i] = 100.0;
        wbdc_data_->tau_min[i] = -100.0;
    }

    wbdc_rotor_ = new WBDC_Rotor(act_list);
    wbdc_rotor_data_ = new WBDC_Rotor_ExtraData();
    wbdc_rotor_data_->A_rotor = 
        dynacore::Matrix::Zero(mercury::num_qdot, mercury::num_qdot);

    wbdc_rotor_data_->cost_weight = 
        dynacore::Vector::Constant(
                com_task_->getDim() + 
                double_contact_->getDim(), 100.0);

    wbdc_rotor_data_->cost_weight[0] = 0.0001; // X
    wbdc_rotor_data_->cost_weight[1] = 0.0001; // Y
    wbdc_rotor_data_->cost_weight[5] = 0.0001; // Yaw
    
    wbdc_rotor_data_->cost_weight.tail(double_contact_->getDim()) = 
        dynacore::Vector::Constant(double_contact_->getDim(), 1.0);
    wbdc_rotor_data_->cost_weight[com_task_->getDim() + 2]  = 0.001; // Fr_z
    wbdc_rotor_data_->cost_weight[com_task_->getDim() + 5]  = 0.001; // Fr_z

   sp_ = Mercury_StateProvider::getStateProvider();

    printf("[Contact Transition CoM Ctrl] Constructed\n");
}

ContactTransCoMCtrl::~ContactTransCoMCtrl(){
    delete com_task_;
    delete double_contact_;
    delete wbdc_;
    delete wbdc_data_;
}

void ContactTransCoMCtrl::OneStep(dynacore::Vector & gamma){
    _PreProcessing_Command();
    state_machine_time_ = sp_->curr_time_ - ctrl_start_time_;
    gamma.setZero();
    _double_contact_setup();
    _com_task_setup();
    //_com_ctrl(gamma);
    _com_ctrl_wbdc_rotor(gamma);

    _PostProcessing_Command();
}

void ContactTransCoMCtrl::_com_ctrl(dynacore::Vector & gamma){

   gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2); 
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

    //wbdc_->UpdateSetting(A_, Ainv_, coriolis_, grav_);
    //wbdc_->MakeTorque(task_list_, contact_list_, gamma, wbdc_data_);

    for(int i(0); i<6; ++i)
       sp_->reaction_forces_[i] = 
            wbdc_data_->opt_result_[task_list_[0]->getDim() + i];
}
void ContactTransCoMCtrl::_com_ctrl_wbdc_rotor(dynacore::Vector & gamma){
    
#if MEASURE_TIME_WBDC 
    static int time_count(0);
    time_count++;
    std::chrono::high_resolution_clock::time_point t1 
        = std::chrono::high_resolution_clock::now();
#endif
   gamma = dynacore::Vector::Zero(mercury::num_act_joint * 2); 
    
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

    gamma.head(mercury::num_act_joint) = fb_cmd;
    gamma.tail(mercury::num_act_joint) = wbdc_rotor_data_->cmd_ff;

#if MEASURE_TIME_WBDC 
    std::chrono::high_resolution_clock::time_point t2 
        = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span1 
        = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
    if(time_count%500 == 1){
        std::cout << "[com ctrl] WBDC_Rotor took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
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


void ContactTransCoMCtrl::_com_task_setup(){
    dynacore::Vector pos_des(3 + 4);
    dynacore::Vector vel_des(com_task_->getDim());
    dynacore::Vector acc_des(com_task_->getDim());
    pos_des.setZero(); vel_des.setZero(); acc_des.setZero();

    // CoM Pos
    pos_des.head(3) = ini_com_pos_;
    if(b_set_height_target_){
        pos_des[2] = dynacore::smooth_changing(ini_com_pos_[2], des_com_height_, end_time_, state_machine_time_);
        vel_des[2] = dynacore::smooth_changing_vel(ini_com_pos_[2], des_com_height_, end_time_, state_machine_time_);
        acc_des[2] = dynacore::smooth_changing_acc(ini_com_pos_[2], des_com_height_, end_time_, state_machine_time_);
    }

    // Orientation
    dynacore::Vect3 rpy_des;
    dynacore::Quaternion quat_des;
    rpy_des.setZero();

    dynacore::convert(rpy_des, quat_des);
    pos_des[3] = quat_des.w();
    pos_des[4] = quat_des.x();
    pos_des[5] = quat_des.y();
    pos_des[6] = quat_des.z();

    com_task_->UpdateTask(&(pos_des), vel_des, acc_des);

    // set relaxed op direction
    // cost weight setup
    // bool b_height_relax(false);
    bool b_height_relax(true);
    if(b_height_relax){
        std::vector<bool> relaxed_op(com_task_->getDim(), true);
        com_task_->setRelaxedOpCtrl(relaxed_op);

        int prev_size(wbdc_data_->cost_weight.rows());
        wbdc_data_->cost_weight.conservativeResize( prev_size + 6);
        wbdc_data_->cost_weight[prev_size] = 0.0001;
        wbdc_data_->cost_weight[prev_size+1] = 0.0001;
        wbdc_data_->cost_weight[prev_size+2] = 10.;
        wbdc_data_->cost_weight[prev_size+3] = 10.;
        wbdc_data_->cost_weight[prev_size+4] = 10.;
        wbdc_data_->cost_weight[prev_size+5] = 0.001;
    }

    // Push back to task list
    task_list_.push_back(com_task_);
    // dynacore::pretty_print(wbdc_data_->cost_weight,std::cout, "cost weight");
}

void ContactTransCoMCtrl::_double_contact_setup(){

    ((DoubleTransitionContact*)double_contact_)->
        setFzUpperLimit(min_rf_z_ + 
                state_machine_time_/end_time_ * (max_rf_z_ - min_rf_z_));
    double_contact_->UpdateContactSpec();

    contact_list_.push_back(double_contact_);
    wbdc_data_->cost_weight = dynacore::Vector::Zero(double_contact_->getDim());
    for(int i(0); i<double_contact_->getDim(); ++i){
        wbdc_data_->cost_weight[i] = 1.;
    }
    wbdc_data_->cost_weight[2] = 0.0001;
    wbdc_data_->cost_weight[5] = 0.0001;
}

void ContactTransCoMCtrl::FirstVisit(){
    // printf("[ContactTransCoM] Start\n");
    robot_sys_->getCoMPosition(ini_com_pos_);
    ctrl_start_time_ = sp_->curr_time_;
}

void ContactTransCoMCtrl::LastVisit(){
    // printf("[ContactTransCoM] End\n");
}

bool ContactTransCoMCtrl::EndOfPhase(){
    if(state_machine_time_ > end_time_){
        return true;
    }
    return false;
}
void ContactTransCoMCtrl::CtrlInitialization(const std::string & setting_file_name){
    robot_sys_->getCoMPosition(ini_com_pos_);

    ParamHandler handler(MercuryConfigPath + setting_file_name + ".yaml");
    handler.getValue("max_rf_z", max_rf_z_);
    handler.getValue("min_rf_z", min_rf_z_);

    // Feedback Gain
    std::vector<double> tmp_vec;
    handler.getVector("Kp", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMOriTask*)com_task_)->Kp_vec_[i] = tmp_vec[i];
    }
    handler.getVector("Kd", tmp_vec);
    for(int i(0); i<tmp_vec.size(); ++i){
        ((CoMOriTask*)com_task_)->Kd_vec_[i] = tmp_vec[i];
    }
}
