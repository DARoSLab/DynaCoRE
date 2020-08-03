#include "SingleSteppingTest.hpp"
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>

#include <SagitP3_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <SagitP3_Controller/CtrlSet/BodyCtrl.hpp>
#include <SagitP3_Controller/CtrlSet/SingleContactTransCtrl.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <Utils/DataManager.hpp>

#include <SagitP3/SagitP3_Model.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>
#include <SagitP3_Controller/CtrlSet/FootSwingCtrl.hpp>

SingleSteppingTest::SingleSteppingTest(RobotSystem* robot):Test(robot),
    num_step_(0)
{
    sp_ = SagitP3_StateProvider::getStateProvider();
    robot_sys_ = robot;
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);
    _SettingParameter();
    
    sp_->stance_foot_ = contact_pt_;
    phase_ = SSteppingPhase::lift_up;
    state_list_.clear();

    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(swing_start_trans_ctrl_);
    state_list_.push_back(swing_ctrl_);
    state_list_.push_back(swing_end_trans_ctrl_);

    DataManager::GetDataManager()->RegisterData(
            &(((FootSwingCtrl*)swing_ctrl_)->curr_foot_pos_des_), 
            VECT3, "foot_pos_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((FootSwingCtrl*)swing_ctrl_)->curr_foot_vel_des_), 
            VECT3, "foot_vel_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((FootSwingCtrl*)swing_ctrl_)->curr_foot_acc_des_), 
            VECT3, "foot_acc_des", 3);

    printf("[Walking  Test] Constructed\n");
}

SingleSteppingTest::~SingleSteppingTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void SingleSteppingTest::TestInitialization(){
    // Yaml file name
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    body_fix_ctrl_->CtrlInitialization("CTRL_stance");
    // Transition
    swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
    // Swing
    swing_ctrl_->CtrlInitialization("CTRL_stepping_swing");
}

int SingleSteppingTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
    printf("next phase: %d\n", next_phase);
    if(next_phase == SSteppingPhase::NUM_SingleStepping_PHASE) {
        return SSteppingPhase::double_contact_1;
    }
    else{ 
        return next_phase; 
    }
}

void SingleSteppingTest::_SettingParameter(){
    // Setting Parameters
    ParamHandler handler(SagitP3ConfigPath"TEST_single_stepping.yaml");

    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;

    handler.getString("stance_leg", tmp_str);
    if( tmp_str == "left" ){
        contact_pt_ = sagitP3_link::l_foot;
        swing_pt_ = sagitP3_link::r_foot;
    }else if (tmp_str == "right"){
        contact_pt_ = sagitP3_link::r_foot;
        swing_pt_ = sagitP3_link::l_foot;
    }else{
        printf("Not valid stance leg\n"); exit(0);
    }
    swing_ctrl_ = 
        new FootSwingCtrl(robot_sys_, swing_pt_);
    swing_start_trans_ctrl_ = new SingleContactTransCtrl(robot_sys_, swing_pt_, false);
    swing_end_trans_ctrl_ = new SingleContactTransCtrl(robot_sys_, swing_pt_, true);


    // CoM Height
    handler.getValue("body_height", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

    ((SingleContactTransCtrl*)swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)swing_end_trans_ctrl_)->setStanceHeight(tmp);

    ((FootSwingCtrl*)swing_ctrl_)->setStanceHeight(tmp);

    //// Timing Setup
    handler.getValue("body_lifting_time", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

    // Stance Time
    handler.getValue("stance_time", tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceTime(tmp);

    // Swing & prime Time
    handler.getValue("swing_time", tmp);
    ((FootSwingCtrl*)swing_ctrl_)->setSwingTime(tmp);

    // Transition Time
    handler.getValue("st_transition_time", tmp);
    ((SingleContactTransCtrl*)swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)swing_end_trans_ctrl_)->setTransitionTime(tmp);

    printf("[Walking Body Test] Complete to Setup Parameters\n");
}
