#include "ConfigStanceSwingTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransConfigCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ConfigBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ConfigBodyFootCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionConfigCtrl.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

ConfigStanceSwingTest::ConfigStanceSwingTest(RobotSystem* robot):Test(robot){
    sp_ = Mercury_StateProvider::getStateProvider();
    sp_->global_pos_local_[1] = 0.15;
    robot_sys_ = robot;
    phase_ = ConfigStanceSwingPhase::stance_swing_initiation;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath"TEST_config_stance_swing.yaml");
    std::string tmp_str;
    handler.getString("swing_foot", tmp_str);
    if(tmp_str == "right"){
        swing_foot_ = mercury_link::rightFoot;
        sp_->stance_foot_ = mercury_link::leftFoot;
    }else if(tmp_str == "left"){
        swing_foot_ = mercury_link::leftFoot;
        sp_->stance_foot_ = mercury_link::rightFoot;
    }else {
        printf("[Config Stance Swing Test] Incorrect Foot Setting\n");
        exit(0);
    }
    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new ContactTransConfigCtrl(robot);
    config_body_fix_ctrl_ = new ConfigBodyCtrl(robot);

    config_swing_start_trans_ctrl_ = 
        new TransitionConfigCtrl(robot, swing_foot_, false);
    config_swing_ctrl_ = new ConfigBodyFootCtrl(robot, swing_foot_);

    state_list_.clear();
    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(config_body_fix_ctrl_);
    state_list_.push_back(config_swing_start_trans_ctrl_);
    state_list_.push_back(config_swing_ctrl_);

    _SettingParameter(handler);
    printf("[Config Stance Swing Test] Constructed\n");
}

ConfigStanceSwingTest::~ConfigStanceSwingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
    state_list_.clear();
}

void ConfigStanceSwingTest::TestInitialization(){
    // Yaml file name
    jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    config_body_fix_ctrl_->CtrlInitialization("CTRL_fix_config");
    // Transition
    config_swing_start_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
    // Swing
    config_swing_ctrl_->CtrlInitialization("CTRL_config_stance_swing");
}

int ConfigStanceSwingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
     printf("next phase: %i\n", next_phase);

    if(next_phase == ConfigStanceSwingPhase::NUM_STANCE_SWING_PHASE) {
        printf("[Body Stance Swing Test] No more state \n");
        exit(0);
    }
    else{ return next_phase; }
}

void ConfigStanceSwingTest::_SettingParameter(ParamHandler& handler){
    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;
    //// Posture Setup
    // Initial JPos
    handler.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
    // Body Height
    handler.getValue("body_height", tmp);
    ((ContactTransConfigCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((ConfigBodyCtrl*)config_body_fix_ctrl_)->setStanceHeight(tmp);

    ((TransitionConfigCtrl*)config_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setStanceHeight(tmp);

    //// Timing Setup
    handler.getValue("jpos_initialization_time", tmp);
    ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
    handler.getValue("body_lifting_time", tmp);
    ((ContactTransConfigCtrl*)body_up_ctrl_)->setStanceTime(tmp);
    handler.getValue("transition_time", tmp);
    ((TransitionConfigCtrl*)config_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    
    // Stance Time
    handler.getValue("stance_time", tmp);
    ((ConfigBodyCtrl*)config_body_fix_ctrl_)->setStanceTime(tmp);

    // Swing
    handler.getValue("swing_duration", tmp);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setSwingTime(tmp);
    handler.getValue("moving_preparation_time", tmp);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setMovingTime(tmp);
    handler.getValue("swing_height", tmp);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setSwingHeight(tmp);

    handler.getVector("amplitude", tmp_vec);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setAmplitude(tmp_vec);
    handler.getVector("frequency", tmp_vec);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setFrequency(tmp_vec);
    handler.getVector("phase", tmp_vec);
    ((ConfigBodyFootCtrl*)config_swing_ctrl_)->setPhase(tmp_vec);

    printf("[Body Stance Swing Test] Complete to Setup Parameters\n");
}
