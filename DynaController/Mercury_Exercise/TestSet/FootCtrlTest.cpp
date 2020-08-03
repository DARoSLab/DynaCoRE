#include "FootCtrlTest.hpp"
#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/FootCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>

FootCtrlTest::FootCtrlTest(RobotSystem* robot):Test(robot){
    phase_ = 0;
    state_list_.clear();

    ParamHandler handle(MercuryConfigPath"TEST_foot_pos_ctrl.yaml");
    std::string tmp_string;

    // Swing foot
    handle.getString("swing_foot", tmp_string);
    if(tmp_string == "right")
        swing_foot_ = mercury_link::rightFoot;
    else if(tmp_string == "left")
        swing_foot_ = mercury_link::leftFoot;
    else
        printf("[Foot test] Wrong Foot Name\n");

    jpos_ini_ = new JPosTargetCtrl(robot);
    foot_ctrl_= new FootCtrl(robot, swing_foot_);

    state_list_.push_back(jpos_ini_);
    state_list_.push_back(foot_ctrl_);

    if(tmp_string == "right"){
        DataManager::GetDataManager()->RegisterData(
                &(((FootCtrl*)foot_ctrl_)->foot_pos_des_), 
                DYN_VEC, "rfoot_pos_des", 3);
        DataManager::GetDataManager()->RegisterData(
                &(((FootCtrl*)foot_ctrl_)->foot_vel_des_), 
                DYN_VEC, "rfoot_vel_des", 3);
    }else{
        DataManager::GetDataManager()->RegisterData(
                &(((FootCtrl*)foot_ctrl_)->foot_pos_des_), 
                DYN_VEC, "lfoot_pos_des", 3);
        DataManager::GetDataManager()->RegisterData(
                &(((FootCtrl*)foot_ctrl_)->foot_vel_des_), 
                DYN_VEC, "lfoot_vel_des", 3);
    }

    _ParameterCtrlSetting();
    printf("[Foot Ctrl Test] Constructed\n");
}
FootCtrlTest::~FootCtrlTest(){
    delete jpos_ini_;
    delete foot_ctrl_;
    state_list_.clear();
}
void FootCtrlTest::TestInitialization(){
    jpos_ini_->CtrlInitialization("CTRL_jpos_initialization");
    foot_ctrl_->CtrlInitialization("CTRL_foot_ctrl");
}

int FootCtrlTest::_NextPhase(const int & phase){
    int nx_phase = phase + 1;
    if(phase == NUM_FOOT_TEST_PHASE){
        nx_phase = FOOT_TEST_SWING;
    }
    return nx_phase;
}

void FootCtrlTest::_ParameterCtrlSetting(){
    ParamHandler handle(MercuryConfigPath"TEST_foot_pos_ctrl.yaml");
    std::vector<double> tmp_vec;
    double tmp_value;

    // Initial posture setup
    handle.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ini_)->setTargetPosition(tmp_vec);
    ((FootCtrl*)foot_ctrl_)->setPosture(tmp_vec);


    // JPos initialization
    handle.getValue("initialization_time", tmp_value);
    ((JPosTargetCtrl*)jpos_ini_)->setMovingTime(tmp_value);

    // Foot Control Setup
    handle.getValue("swing_test_time", tmp_value);
    ((FootCtrl*)foot_ctrl_)->setMovingTime(tmp_value);
    handle.getVector("amplitude", tmp_vec);
    ((FootCtrl*)foot_ctrl_)->setAmplitude(tmp_vec);
    handle.getVector("frequency", tmp_vec);
    ((FootCtrl*)foot_ctrl_)->setFrequency(tmp_vec);
    handle.getVector("phase", tmp_vec);
    ((FootCtrl*)foot_ctrl_)->setPhase(tmp_vec);

    printf("[Foot Test] Parameter Setting is Done.\n");
}
