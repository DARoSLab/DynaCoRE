#include "Mercury_interface.hpp"
#include <stdio.h>
#include <math.h>
#include <string>
#include <Utils/utilities.hpp>
#include <Utils/pseudo_inverse.hpp>
#include <Utils/DataManager.hpp>
#include <Utils/wrap_eigen.hpp>
#include "Mercury_StateProvider.hpp"
#include "Mercury_StateEstimator.hpp"
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>

// Test SET LIST
// Basic Test
#include <Mercury_Controller/TestSet/JointCtrlTest.hpp>
#include <Mercury_Controller/TestSet/FootCtrlTest.hpp>

// Walking Test
#include <Mercury_Controller/TestSet/WalkingTest.hpp>
#include <Mercury_Controller/TestSet/WalkingBodyTest.hpp>
#include <Mercury_Controller/TestSet/WalkingConfigTest.hpp>

// Body Ctrl Test
#include <Mercury_Controller/TestSet/CoMCtrlTest.hpp>
#include <Mercury_Controller/TestSet/BodyCtrlTest.hpp>
#include <Mercury_Controller/TestSet/BodyJPosCtrlTest.hpp>

// Stance and Swing Test
#include <Mercury_Controller/TestSet/ConfigStanceSwingTest.hpp>
#include <Mercury_Controller/TestSet/CoMStanceSwingTest.hpp>
#include <Mercury_Controller/TestSet/BodyStanceSwingTest.hpp>

#define MEASURE_TIME 0
#if MEASURE_TIME
#include <chrono>
#endif

Mercury_interface::Mercury_interface():
    interface(),
    torque_command_(mercury::num_act_joint* 2),
    test_command_(mercury::num_act_joint),
    sensed_torque_(mercury::num_act_joint),
    torque_limit_max_(mercury::num_act_joint),
    torque_limit_min_(mercury::num_act_joint),
    motor_current_(mercury::num_act_joint),
    bus_current_(mercury::num_act_joint),
    bus_voltage_(mercury::num_act_joint),
    jjvel_(mercury::num_act_joint),
    mjpos_(mercury::num_act_joint),
    waiting_count_(10)
{
    robot_sys_ = new Mercury_Model();
    sensed_torque_.setZero();
    torque_command_.setZero();
    motor_current_.setZero();
    bus_current_.setZero();
    bus_voltage_.setZero();
    test_command_.setZero();
    jjvel_.setZero();
    mjpos_.setZero();

    sp_ = Mercury_StateProvider::getStateProvider();
    state_estimator_ = new Mercury_StateEstimator(robot_sys_);  

    DataManager::GetDataManager()->RegisterData(
            &running_time_, DOUBLE, "running_time");
    DataManager::GetDataManager()->RegisterData(
            &sensed_torque_, DYN_VEC, "torque", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &torque_command_, DYN_VEC, "command", mercury::num_act_joint * 2);
    DataManager::GetDataManager()->RegisterData(
            &motor_current_, DYN_VEC, "motor_current", mercury::num_act_joint);
    //DataManager::GetDataManager()->RegisterData(
            //&jjvel_, DYN_VEC, "joint_jvel", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &mjpos_, DYN_VEC, "motor_jpos", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &bus_current_, DYN_VEC, "bus_current", mercury::num_act_joint);
    DataManager::GetDataManager()->RegisterData(
            &bus_voltage_, DYN_VEC, "bus_voltage", mercury::num_act_joint);

    _ParameterSetting();
    printf("[Mercury_interface] Contruct\n");
}

Mercury_interface::~Mercury_interface(){
    delete test_;
}

void Mercury_interface::GetCommand( void* _data,
        std::vector<double> & command){
    
    command.resize(mercury::num_act_joint*2, 0.);
    Mercury_SensorData* data = ((Mercury_SensorData*)_data);
    
    if(!_Initialization(data)){
#if MEASURE_TIME
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
#endif
        state_estimator_->Update(data);
        // Calcualate Torque
        test_->getTorqueInput(test_command_);

#if MEASURE_TIME
        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span1 = std::chrono::duration_cast< std::chrono::duration<double> >(t2 - t1);
        if(count_%1000 == 1){
            std::cout << "[Mercury_interface] All process took me " << time_span1.count()*1000.0 << "ms."<<std::endl;
        }
#endif
    }

    /// Begin of Torque Limit && NAN command
    static bool isTurnoff(false);
    static bool isTurnoff_forever(false);

    if(isTurnoff_forever){
        test_command_.setZero();
    } else{
        for(int i(0); i<test_command_.size(); ++i){
            if(std::isnan(test_command_[i])){ 
                isTurnoff_forever = true;
                printf("[Interface] There is nan value in command\n");
                test_command_.setZero();
                // TEST
                exit(0);
            }
        }
    }


    for (int i(0); i<mercury::num_act_joint; ++i){
        if( (test_command_[i] > torque_limit_max_[i]) ){
            //if (count_ % 100 == 0) {
            //printf("%i th torque is too large: %f\n", i, torque_command_[i]);
            //}
            torque_command_[i] = torque_limit_max_[i];
            isTurnoff = true;
            //exit(0);
        }else if(test_command_[i]< torque_limit_min_[i]) {
            //if (count_ % 100 == 0) {
            //printf("%i th torque is too small: %f\n", i, torque_command_[i]);
            //}
            torque_command_[i] = torque_limit_min_[i];
            isTurnoff = true;
            //exit(0);
        } else{
            torque_command_[i] = test_command_[i];
        }
        command[i] = torque_command_[i];
        sensed_torque_[i] = data->jtorque[i];
        motor_current_[i] = data->motor_current[i];
        bus_current_[i] = data->bus_current[i];
        bus_voltage_[i] = data->bus_voltage[i];
        jjvel_[i] = data->joint_jvel[i];
        mjpos_[i] = data->motor_jpos[i];
    }
    if (isTurnoff) {
        //torque_command_.setZero();
        for(int i(0); i<mercury::num_act_joint; ++i){
            command[i] = 0.;
        }
        isTurnoff = false;
    }
    /// End of Torque Limit Check


    // IF torque feedforward control is computed
    if( test_command_.rows() == 2* mercury::num_act_joint ){
        /// Begin of Torque Limit
        int k(0);
        for (int i(mercury::num_act_joint); i<mercury::num_act_joint*2; ++i){
            k = i - mercury::num_act_joint;
            if(test_command_[i] > torque_limit_max_[k]){
                torque_command_[i] = torque_limit_max_[k];
                isTurnoff = true;
            }else if(test_command_[i]< torque_limit_min_[k]){
                torque_command_[i] = torque_limit_min_[k];
                isTurnoff = true;
            } else{
                torque_command_[i] = test_command_[i];
            }
            command[i] = torque_command_[i];
        }
        if (isTurnoff) {
            //torque_command_.setZero();
            for(int i(mercury::num_act_joint); i<2*mercury::num_act_joint; ++i){
                command[i] = 0.;
            }
            isTurnoff = false;
        }
        /// End of Torque Limit Check
    } else {
        for(int i(0); i<mercury::num_act_joint; ++i){
            command[i + mercury::num_act_joint] = torque_command_[i];
        }
    }

    running_time_ = (double)(count_) * mercury::servo_rate;
    ++count_;
    // When there is sensed time
    sp_->curr_time_ = running_time_;

    // TEST
    //dynacore::Matrix Jfoot, Jfoot_inv;
    //dynacore::Vect3 foot_pos;
    //dynacore::Vector config = sp_->Q_;
    //for(int i(0); i<3; ++i){
    //config[i] = 0.;
    //}

    //int link_id = mercury_link::leftFoot;
    //robot_sys_->getPos(link_id, foot_pos);
    //robot_sys_->getFullJacobian(link_id, Jfoot);
    //dynacore::pseudoInverse(Jfoot.block(3,0, 3, mercury::num_qdot), 0.0001, Jfoot_inv);
  
    //dynacore::pretty_print(sp_->Q_, std::cout, "config");
    //dynacore::pretty_print(foot_pos, std::cout, "foot pos");
    //dynacore::pretty_print(Jfoot, std::cout, "JFoot");
    //dynacore::pretty_print(Jfoot_inv, std::cout, "JFoot inv");



    //if(count_%500== 1){
    //dynacore::pretty_print(data->imu_ang_vel, "imu ang vel", 3);
    //dynacore::pretty_print(data->imu_acc, "imu acc", 3);
    //dynacore::Quaternion ori = sp_->body_ori_;
    //dynacore::pretty_print(ori, std::cout, "estimated_quat");

    //double yaw, pitch, roll;
    //dynacore::convert(ori, yaw, pitch, roll);
    //printf("rpy: %f, %f, %f\n", roll, pitch, yaw);
    //printf("\n");
    //}

     //if(count_%100== 1){
       //dynacore::pretty_print(global_ori_, std::cout, "sim global quat");
       //dynacore::pretty_print(sp_->body_ori_, std::cout, "estimated_quat");
       //printf("\n");
     //}
}
void Mercury_interface::GetReactionForce(std::vector<dynacore::Vect3> & reaction_force ){
    reaction_force.resize(2);
    for(int i(0); i<2; ++i){
        for(int j(0); j<3; ++j){
            reaction_force[i][j] = sp_->reaction_forces_[j];
        }
    }
}

bool Mercury_interface::_Initialization(Mercury_SensorData* data){
    if(count_ < waiting_count_){
        torque_command_.setZero();
        state_estimator_->Initialization(data);
        test_->TestInitialization();

        if(fabs(data->imu_acc[2]) < 0.00001){
            waiting_count_ = 10000000;
        }else{
            waiting_count_ = 10;
        }

        DataManager::GetDataManager()->start();
        return true;
    }
    return false;
}

void Mercury_interface::_ParameterSetting(){
    ParamHandler handler(MercuryConfigPath"INTERFACE_setup.yaml");

    std::string tmp_string;
    bool b_tmp;
    // Test SETUP
    handler.getString("test_name", tmp_string);
    // Basic Test ***********************************
    if(tmp_string == "joint_ctrl_test"){
        test_ = new JointCtrlTest(robot_sys_);
    }else if(tmp_string == "foot_ctrl_test"){
        test_ = new FootCtrlTest(robot_sys_);
    // Walking Test ***********************************
    }else if(tmp_string == "walking_com_test"){
        test_ = new WalkingTest(robot_sys_);
    }else if(tmp_string == "walking_body_test"){
        test_ = new WalkingBodyTest(robot_sys_);
    }else if(tmp_string == "walking_config_test"){
        test_ = new WalkingConfigTest(robot_sys_);
    // Body Ctrl Test ***********************************
    }else if(tmp_string == "body_ctrl_test"){
        test_ = new BodyCtrlTest(robot_sys_);
    }else if(tmp_string == "com_ctrl_test"){
        test_ = new CoMCtrlTest(robot_sys_);
    }else if(tmp_string == "body_jpos_ctrl_test"){
        test_ = new BodyJPosCtrlTest(robot_sys_);    
    // Stance and Swing Test ***********************************
    }else if(tmp_string == "body_stance_swing_test"){
        test_ = new BodyStanceSwingTest(robot_sys_);
    }else if(tmp_string == "com_stance_swing_test"){
        test_ = new CoMStanceSwingTest(robot_sys_);
    }else if(tmp_string == "config_stance_swing_test"){
        test_ = new ConfigStanceSwingTest(robot_sys_);
    }else {
        printf("[Interfacce] There is no test matching with the name\n");
        exit(0);
    }

    // State Estimator Setup
    handler.getString("base_condition", tmp_string);
    if(tmp_string == "floating")
        state_estimator_->setFloatingBase(base_condition::floating);
    else if(tmp_string == "fixed")
        state_estimator_->setFloatingBase(base_condition::fixed);
    else if(tmp_string == "lying")
        state_estimator_->setFloatingBase(base_condition::lying);
    else
        printf("[Interface] Error: No proper base condition\n");

    // Torque limit
    handler.getVector("torque_max", torque_limit_max_);
    handler.getVector("torque_min", torque_limit_min_);

    printf("[Mercury_interface] Parameter setup is done\n");
}
