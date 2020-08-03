#include "TELLO_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "TELLO_StateProvider.hpp"
#include "TELLO_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <TELLO/TELLO_Model.hpp>

// Walking Test
#include <TELLO_Controller/TestSet/WalkingTest.hpp>

// Body Ctrl Test
#include <TELLO_Controller/TestSet/BodyCtrlTest.hpp>

TELLO_interface::TELLO_interface():
    interface(),
    jjvel_(tello::num_act_joint),
    jjpos_(tello::num_act_joint),
    jtorque_(tello::num_act_joint),
    torque_command_(tello::num_act_joint),
    jpos_command_(tello::num_act_joint),
    jvel_command_(tello::num_act_joint),
    waiting_count_(2)
{
    robot_sys_ = new TELLO_Model();
    jjvel_.setZero();
    jjpos_.setZero();
    jtorque_.setZero();

    torque_command_.setZero();
    jpos_command_.setZero();
    jvel_command_.setZero();
 
    test_cmd_ = new TELLO_Command();
    sp_ = TELLO_StateProvider::getStateProvider();
    state_estimator_ = new TELLO_StateEstimator(robot_sys_);  
    
    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &jjpos_, DYN_VEC, "jjpos", tello::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jjvel_, DYN_VEC, "jjvel", tello::num_act_joint);
     DataManager::GetDataManager()->RegisterData(
            &jtorque_, DYN_VEC, "torque", tello::num_act_joint);
  
    DataManager::GetDataManager()->RegisterData(
            &jpos_command_, DYN_VEC, "jpos_des", tello::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &jvel_command_, DYN_VEC, "jvel_des", tello::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", tello::num_act_joint);

    _ParameterSetting();
    
    printf("[TELLO_interface] Contruct\n");
}

TELLO_interface::~TELLO_interface(){
    delete test_;
}

void TELLO_interface::GetCommand( void* _data, void* _command){
    TELLO_Command* cmd = ((TELLO_Command*)_command);
    TELLO_SensorData* data = ((TELLO_SensorData*)_data);

    if(!_Initialization(data)){
        state_estimator_->Update(data);
        test_->getCommand(test_cmd_);
    }
    
    // Update Command (and Data)
    for(int i(0); i<tello::num_act_joint; ++i){
        torque_command_[i] = test_cmd_->jtorque_cmd[i];
        jpos_command_[i] = test_cmd_->jpos_cmd[i];
        jvel_command_[i] = test_cmd_->jvel_cmd[i];

        jjvel_[i] = data->jvel[i];
        jjpos_[i] = data->jpos[i];
        jtorque_[i] = data->jtorque[i];
    }

    for(int i(0); i< tello::num_act_joint; ++i){
        cmd->jtorque_cmd[i] = torque_command_[i];
        cmd->jpos_cmd[i] = jpos_command_[i];
        cmd->jvel_cmd[i] = jvel_command_[i];
    }

    running_time_ = (double)(count_) * tello::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;
    
    ////// Stepping forward
    //double walking_start(3.);
    //double walking_duration(8.);
    //double walking_distance(2.5);
    //if(sp_->curr_time_ > walking_start){
        //double walking_time = sp_->curr_time_ - walking_start;
        //sp_->des_location_[0] = walking_distance * 
            //walking_time/walking_duration;
            //(1-cos(walking_time/walking_duration * M_PI))/2.;
    //}
    //if(sp_->curr_time_ > walking_start + walking_duration){
        //sp_->des_location_[0] = walking_distance;
    //}
}

bool TELLO_interface::_Initialization(TELLO_SensorData* data){
    static bool test_initialized(false);
    if(!test_initialized) {
        test_->TestInitialization();
        test_initialized = true;
        printf("[TELLO Interface] Test initialization is done\n");
    }
    if(count_ < waiting_count_){
        for(int i(0); i<tello::num_act_joint; ++i){
            test_cmd_->jtorque_cmd[i] = 0.;
            test_cmd_->jpos_cmd[i] = data->jpos[i];
            test_cmd_->jvel_cmd[i] = 0.;
        }
        state_estimator_->Initialization(data);

        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void TELLO_interface::_ParameterSetting(){
    ParamHandler handler(TELLOConfigPath"INTERFACE_setup.yaml");
    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Walking Test ***********************************
    if(tmp_string == "walking_config_test"){
        test_ = new WalkingTest(robot_sys_);
        // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyCtrlTest(robot_sys_);    
        // Stance and Swing Test ***********************************
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }
    printf("[TELLO_interface] Parameter setup is done\n");
}
