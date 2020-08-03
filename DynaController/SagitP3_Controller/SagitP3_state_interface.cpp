#include "SagitP3_state_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "SagitP3_StateProvider.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <SagitP3/SagitP3_Model.hpp>

// Test SET LIST
#include <SagitP3_Controller/TestSet/JointCtrlTest.hpp>
#include <SagitP3_Controller/TestSet/BodyCtrlTest.hpp>
#include <SagitP3_Controller/TestSet/WalkingConfigTest.hpp>
#include <SagitP3_Controller/TestSet/SingleSteppingTest.hpp>

SagitP3_state_interface::SagitP3_state_interface():
    interface(),
    torque_command_(sagitP3::num_act_joint),
    jpos_command_(sagitP3::num_act_joint),
    jvel_command_(sagitP3::num_act_joint)
{
    robot_sys_ = new SagitP3_Model();
    jpos_command_.setZero();
    jvel_command_.setZero();

    test_cmd_ = new SagitP3_Command();
    sp_ = SagitP3_StateProvider::getStateProvider();
    
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", sagitP3::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", sagitP3::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", sagitP3::num_act_joint);

    _ParameterSetting();
    
    test_->TestInitialization();
        DataManager::GetDataManager()->start();
    printf("[SagitP3_state_interface] Contruct\n");
}

SagitP3_state_interface::~SagitP3_state_interface(){
    delete robot_sys_;
    delete test_;
}

void SagitP3_state_interface::GetCommand( void* _data, void* _command){
    SagitP3_Command* cmd = ((SagitP3_Command*)_command);
    SagitP3_StateData* data = ((SagitP3_StateData*)_data);

    for(int i(0); i<sagitP3::num_qdot; ++i){
        sp_->Q_[i] = data->q[i];
        sp_->Qdot_[i] = data->qdot[i];
    }
    sp_->Q_[sagitP3_joint::virtual_Rw] = data->q[sagitP3_joint::virtual_Rw];
    robot_sys_->UpdateSystem(sp_->Q_, sp_->Qdot_);
    sp_->SaveCurrentData(robot_sys_);
    //dynacore::pretty_print(sp_->Q_, std::cout, "config");
    //dynacore::pretty_print(sp_->Qdot_, std::cout, "qdot");
 
    test_->getCommand(test_cmd_);

    //dynacore::pretty_print(test_cmd_->jtorque_cmd, "jtorque", sagitP3::num_act_joint);
    // Update Command (and Data)
    for(int i(0); i<sagitP3::num_act_joint; ++i){
        torque_command_[i] = test_cmd_->jtorque_cmd[i];
        jpos_command_[i] = test_cmd_->jpos_cmd[i];
        jvel_command_[i] = test_cmd_->jvel_cmd[i];
    }

    for(int i(0); i< sagitP3::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = torque_command_[i];
        cmd->jpos_cmd[i] = jpos_command_[i];
        cmd->jvel_cmd[i] = jvel_command_[i];
    }

    running_time_ = (double)(count_) * sagitP3::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;
}

void SagitP3_state_interface::_ParameterSetting(){
    ParamHandler handler(SagitP3ConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Basic Test ***********************************
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
    // Walking Test ***********************************
    }else if(tmp_string == "walking_test"){
        test_ = new WalkingConfigTest(robot_sys_);
    // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyCtrlTest(robot_sys_);
        // Stance and Swing Test ***********************************
    }else if(tmp_string == "single_stepping_test"){
        test_ = new SingleSteppingTest(robot_sys_);
     }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    printf("[SagitP3_state_interface] Parameter setup is done\n");
}
