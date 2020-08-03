#include "BodyStanceSwingTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyFootJPosCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCtrl.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

BodyStanceSwingTest::BodyStanceSwingTest(RobotSystem* robot):Test(robot){
    sp_ = Mercury_StateProvider::getStateProvider();
    sp_->global_pos_local_[1] = 0.15;
    robot_sys_ = robot;
    phase_ = BodyStanceSwingPhase::stance_swing_initiation;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath"TEST_body_stance_swing.yaml");
    std::string tmp_str;
    handler.getString("swing_foot", tmp_str);
    if(tmp_str == "right"){
        swing_foot_ = mercury_link::rightFoot;
        sp_->stance_foot_ = mercury_link::leftFoot;
    }else if(tmp_str == "left"){
        swing_foot_ = mercury_link::leftFoot;
        sp_->stance_foot_ = mercury_link::rightFoot;
    }else {
        printf("[Stance Swing Test] Incorrect Foot Setting\n");
        exit(0);
    }
    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new ContactTransBodyCtrl(robot);
    body_fix_ctrl_ = new BodyRxRyRzCtrl(robot);

    swing_start_trans_ctrl_ = 
        new TransitionCtrl(robot, swing_foot_, false);
    jpos_swing_ctrl_ = new BodyFootJPosCtrl(robot, swing_foot_);

    state_list_.clear();
    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(swing_start_trans_ctrl_);
    state_list_.push_back(jpos_swing_ctrl_);

    _SettingParameter(handler);
    printf("[Stance Swing Test] Constructed\n");
}

BodyStanceSwingTest::~BodyStanceSwingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
    state_list_.clear();
}

void BodyStanceSwingTest::TestInitialization(){
    // Yaml file name
    jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
    // Transition
    swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    // Swing
    jpos_swing_ctrl_->CtrlInitialization("CTRL_stance_swing");
}

int BodyStanceSwingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    // printf("next phase: %i\n", next_phase);

    if(next_phase == BodyStanceSwingPhase::NUM_STANCE_SWING_PHASE) {
        printf("[Body Stance Swing Test] No more state \n");
        exit(0);
    }
    else{ return next_phase; }
}

void BodyStanceSwingTest::_SettingParameter(ParamHandler& handler){
    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;
    //// Posture Setup
    // Initial JPos
    handler.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
    // Body Height
    handler.getValue("body_height", tmp);
    ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((BodyRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

    ((TransitionCtrl*)swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setStanceHeight(tmp);

    //// Timing Setup
    handler.getValue("jpos_initialization_time", tmp);
    ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
    handler.getValue("body_lifting_time", tmp);
    ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);
    handler.getValue("transition_time", tmp);
    ((TransitionCtrl*)swing_start_trans_ctrl_)->setTransitionTime(tmp);
    // Stance Time
    handler.getValue("stance_time", tmp);
    ((BodyRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);

    // Swing
    handler.getValue("swing_duration", tmp);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setSwingTime(tmp);
    handler.getValue("moving_preparation_time", tmp);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setMovingTime(tmp);
    handler.getValue("swing_height", tmp);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setSwingHeight(tmp);

    handler.getVector("amplitude", tmp_vec);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setAmplitude(tmp_vec);
    handler.getVector("frequency", tmp_vec);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setFrequency(tmp_vec);
    handler.getVector("phase", tmp_vec);
    ((BodyFootJPosCtrl*)jpos_swing_ctrl_)->setPhase(tmp_vec);

    printf("[Body Stance Swing Test] Complete to Setup Parameters\n");
}
