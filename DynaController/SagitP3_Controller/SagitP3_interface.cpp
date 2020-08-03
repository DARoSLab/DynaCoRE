#include "SagitP3_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "SagitP3_StateProvider.hpp"
#include "SagitP3_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <SagitP3/SagitP3_Model.hpp>

// Test SET LIST
#include <SagitP3_Controller/TestSet/JointCtrlTest.hpp>
#include <SagitP3_Controller/TestSet/BodyCtrlTest.hpp>
#include <SagitP3_Controller/TestSet/WalkingConfigTest.hpp>
#include <SagitP3_Controller/TestSet/SingleSteppingTest.hpp>

SagitP3_interface::SagitP3_interface():
    interface(),
    torque_(sagitP3::num_act_joint),
    jjvel_(sagitP3::num_act_joint),
    jjpos_(sagitP3::num_act_joint),
    torque_command_(sagitP3::num_act_joint),
    jpos_command_(sagitP3::num_act_joint),
    jvel_command_(sagitP3::num_act_joint),
     waiting_count_(5)
{

    robot_sys_ = new SagitP3_Model();
    jjvel_.setZero();
    jjpos_.setZero();
    jvel_command_.setZero();

    test_cmd_ = new SagitP3_Command();
    sp_ = SagitP3_StateProvider::getStateProvider();
    state_estimator_ = new SagitP3_StateEstimator(robot_sys_);  
    
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &jjpos_, DYN_VEC, "jjpos", sagitP3::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjvel_, DYN_VEC, "jjvel", sagitP3::num_act_joint);
     DataManager::GetDataManager()->RegisterData(
            &torque_, DYN_VEC, "torque", sagitP3::num_act_joint);
    
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", sagitP3::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", sagitP3::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", sagitP3::num_act_joint);

    _ParameterSetting();
    
    printf("[SagitP3_interface] Contruct\n");
}

SagitP3_interface::~SagitP3_interface(){
    delete test_;
}

void SagitP3_interface::GetCommand( void* _data, void* _command){
    SagitP3_Command* cmd = ((SagitP3_Command*)_command);
    SagitP3_SensorData* data = ((SagitP3_SensorData*)_data);

    if(!_Initialization(data)){
        state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
    }
    
    // Update Command (and Data)
    for(int i(0); i<sagitP3::num_act_joint; ++i){
        torque_command_[i] = test_cmd_->jtorque_cmd[i];
        jpos_command_[i] = test_cmd_->jpos_cmd[i];
        jvel_command_[i] = test_cmd_->jvel_cmd[i];

        jjvel_[i] = data->jvel[i];
        torque_[i] = data->torque[i];
        jjpos_[i] = data->jpos[i];
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
    // Stepping forward
    double walking_start(3.);
    double walking_duration(7.);
    double walking_distance(2.5);
    if(sp_->curr_time_ > walking_start){
        double walking_time = sp_->curr_time_ - walking_start;
        sp_->des_location_[0] = walking_distance * 
            (1-cos(walking_time/walking_duration * M_PI))/2.;
    }
    if(sp_->curr_time_ > walking_start + walking_duration){
        sp_->des_location_[0] = walking_distance;
    }
}

bool SagitP3_interface::_Initialization(SagitP3_SensorData* data){
   if(count_ < waiting_count_){
        for(int i(0); i<sagitP3::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = 0.;
            test_cmd_->jpos_cmd[i] = data->jpos[i];
            test_cmd_->jvel_cmd[i] = 0.;
        }
        state_estimator_->Initialization(data);
        DataManager::GetDataManager()->start();
        return true;
    }
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[SagitP3 Interface] Test initialization is done\n");
    }
     return false;
}

void SagitP3_interface::_ParameterSetting(){
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
    printf("[SagitP3_interface] Parameter setup is done\n");
}
