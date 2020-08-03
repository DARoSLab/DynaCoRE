#include "CoMStanceSwingTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransCoMCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMFootCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMFootJPosCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCtrl.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

CoMStanceSwingTest::CoMStanceSwingTest(RobotSystem* robot):Test(robot){
    sp_ = Mercury_StateProvider::getStateProvider();
    sp_->global_pos_local_[1] = 0.15;
    robot_sys_ = robot;
    phase_ = CoMStanceSwingPhase::stance_swing_initiation;
    // Setting Parameters
    ParamHandler handler(MercuryConfigPath"TEST_com_stance_swing.yaml");
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
    com_up_ctrl_ = new ContactTransCoMCtrl(robot);
    com_fix_ctrl_ = new CoMzRxRyRzCtrl(robot);

    swing_start_trans_ctrl_ = 
        new TransitionCtrl(robot, swing_foot_, false);
    swing_ctrl_ = 
        new CoMFootCtrl(robot, swing_foot_);
    jpos_swing_ctrl_ = new CoMFootJPosCtrl(robot, swing_foot_);

    state_list_.clear();
    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(com_up_ctrl_);
    state_list_.push_back(com_fix_ctrl_);
    state_list_.push_back(swing_start_trans_ctrl_);
    state_list_.push_back(swing_ctrl_);
    //state_list_.push_back(jpos_swing_ctrl_);

    _SettingParameter(handler);

    if(swing_foot_ == mercury_link::rightFoot){
        DataManager::GetDataManager()->RegisterData(
                &(((CoMFootCtrl*)swing_ctrl_)->curr_foot_pos_des_), 
                VECT3, "rfoot_pos_des", 3);
        DataManager::GetDataManager()->RegisterData(
                &(((CoMFootCtrl*)swing_ctrl_)->curr_foot_vel_des_), 
                VECT3, "rfoot_vel_des", 3);
    } else if (swing_foot_ == mercury_link::leftFoot){
        DataManager::GetDataManager()->RegisterData(
                &(((CoMFootCtrl*)swing_ctrl_)->curr_foot_pos_des_), 
                VECT3, "lfoot_pos_des", 3);

        DataManager::GetDataManager()->RegisterData(
                &(((CoMFootCtrl*)swing_ctrl_)->curr_foot_vel_des_), 
                VECT3, "lfoot_vel_des", 3);
    } else{
        printf("[Stance Swing Test] Incorrect swing foot setup\n");
        exit(0);
    }

    printf("[Stance Swing Test] Constructed\n");
}

CoMStanceSwingTest::~CoMStanceSwingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
    state_list_.clear();
}

void CoMStanceSwingTest::TestInitialization(){
    // Yaml file name
    jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
    com_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    com_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
    // Transition
    swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    // Swing
    swing_ctrl_->CtrlInitialization("CTRL_stance_swing");
    jpos_swing_ctrl_->CtrlInitialization("CTRL_stance_swing");
}

int CoMStanceSwingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    // printf("next phase: %i\n", next_phase);

    if(next_phase == CoMStanceSwingPhase::NUM_STANCE_SWING_PHASE) {
        printf("[Stance Swing Test] No more state \n");
        exit(0);
    }
    else{ return next_phase; }
}

void CoMStanceSwingTest::_SettingParameter(ParamHandler& handler){
    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;
    //// Posture Setup
    // Initial JPos
    handler.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
    // CoM Height
    handler.getValue("com_height", tmp);
    ((ContactTransCoMCtrl*)com_up_ctrl_)->setStanceHeight(tmp);
    ((CoMzRxRyRzCtrl*)com_fix_ctrl_)->setStanceHeight(tmp);

    ((TransitionCtrl*)swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((CoMFootCtrl*)swing_ctrl_)->setStanceHeight(tmp);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setStanceHeight(tmp);

    //// Timing Setup
    handler.getValue("jpos_initialization_time", tmp);
    ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
    handler.getValue("com_lifting_time", tmp);
    ((ContactTransCoMCtrl*)com_up_ctrl_)->setStanceTime(tmp);
    handler.getValue("transition_time", tmp);
    ((TransitionCtrl*)swing_start_trans_ctrl_)->setTransitionTime(tmp);
    // Stance Time
    handler.getValue("stance_time", tmp);
    ((CoMzRxRyRzCtrl*)com_fix_ctrl_)->setStanceTime(tmp);

    // Swing
    handler.getValue("swing_duration", tmp);
    ((CoMFootCtrl*)swing_ctrl_)->setSwingTime(tmp);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setSwingTime(tmp);
    handler.getValue("moving_preparation_time", tmp);
    ((CoMFootCtrl*)swing_ctrl_)->setMovingTime(tmp);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setMovingTime(tmp);
    handler.getValue("swing_height", tmp);
    ((CoMFootCtrl*)swing_ctrl_)->setSwingHeight(tmp);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setSwingHeight(tmp);

    handler.getVector("amplitude", tmp_vec);
    ((CoMFootCtrl*)swing_ctrl_)->setAmplitude(tmp_vec);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setAmplitude(tmp_vec);
    handler.getVector("frequency", tmp_vec);
    ((CoMFootCtrl*)swing_ctrl_)->setFrequency(tmp_vec);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setFrequency(tmp_vec);
    handler.getVector("phase", tmp_vec);
    ((CoMFootCtrl*)swing_ctrl_)->setPhase(tmp_vec);
    ((CoMFootJPosCtrl*)jpos_swing_ctrl_)->setPhase(tmp_vec);

    printf("[Stance Swing Test] Complete to Setup Parameters\n");
}
