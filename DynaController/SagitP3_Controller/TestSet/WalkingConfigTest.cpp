#include "WalkingConfigTest.hpp"
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>

#include <SagitP3_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <SagitP3_Controller/CtrlSet/DoubleContactTransCtrl.hpp>
#include <SagitP3_Controller/CtrlSet/BodyCtrl.hpp>
#include <SagitP3_Controller/CtrlSet/SingleContactTransCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <SagitP3/SagitP3_Model.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>
#include <SagitP3_Controller/CtrlSet/BodyFootPlanningCtrl.hpp>

WalkingConfigTest::WalkingConfigTest(RobotSystem* robot):Test(robot),
    num_step_(0)
{
    sp_ = SagitP3_StateProvider::getStateProvider();
#if (CONFIG_INITIAL_SWING_FOOT == 0)
    sp_->stance_foot_ = sagitP3_link::l_ankle;
#else
    sp_->stance_foot_ = sagitP3_link::r_ankle;
#endif
    sp_->global_pos_local_[1] = 0.15;
    robot_sys_ = robot;
    reversal_planner_ = new Reversal_LIPM_Planner();
    phase_ = WkConfigPhase::initiation;

    state_list_.clear();

    jpos_ctrl_ = new JPosTargetCtrl(robot);
    body_up_ctrl_ = new DoubleContactTransCtrl(robot);
    body_fix_ctrl_ = new BodyCtrl(robot);
    // Swing Controller Selection
    right_swing_ctrl_ = 
        new BodyFootPlanningCtrl(robot_sys_, sagitP3_link::r_ankle, reversal_planner_);
    left_swing_ctrl_ = 
        new BodyFootPlanningCtrl(robot_sys_, sagitP3_link::l_ankle, reversal_planner_);

    // Right
    right_swing_start_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, sagitP3_link::r_ankle, false);
    right_swing_end_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, sagitP3_link::r_ankle, true);
    // Left
    left_swing_start_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, sagitP3_link::l_ankle, false);
    left_swing_end_trans_ctrl_ = 
        new SingleContactTransCtrl(robot, sagitP3_link::l_ankle, true);


    _SettingParameter();

    state_list_.push_back(jpos_ctrl_);
    state_list_.push_back(body_up_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(right_swing_start_trans_ctrl_);
    state_list_.push_back(right_swing_ctrl_);
    state_list_.push_back(right_swing_end_trans_ctrl_);
    state_list_.push_back(body_fix_ctrl_);
    state_list_.push_back(left_swing_start_trans_ctrl_);
    state_list_.push_back(left_swing_ctrl_);
    state_list_.push_back(left_swing_end_trans_ctrl_);


    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_pos_des_), 
            VECT3, "rfoot_pos_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_pos_des_), 
            VECT3, "lfoot_pos_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_vel_des_), 
            VECT3, "rfoot_vel_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_vel_des_), 
            VECT3, "lfoot_vel_des", 3);

    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)right_swing_ctrl_)->curr_foot_acc_des_), 
            VECT3, "rfoot_acc_des", 3);
    DataManager::GetDataManager()->RegisterData(
            &(((SwingPlanningCtrl*)left_swing_ctrl_)->curr_foot_acc_des_), 
            VECT3, "lfoot_acc_des", 3);

    printf("[Walking Config Test] Constructed\n");
}

WalkingConfigTest::~WalkingConfigTest(){
    for(int i(0); i<state_list_.size(); ++i){
        delete state_list_[i];
    }
}

void WalkingConfigTest::TestInitialization(){
    reversal_planner_->PlannerInitialization(SagitP3ConfigPath"PLANNER_velocity_reversal");
    // Yaml file name
    jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
    body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
    body_fix_ctrl_->CtrlInitialization("CTRL_stance");
    // Transition
    right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
    left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
    left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
    // Swing
    right_swing_ctrl_->CtrlInitialization("CTRL_right_walking_swing");
    left_swing_ctrl_->CtrlInitialization("CTRL_left_walking_swing");
}

int WalkingConfigTest::_NextPhase(const int & phase){
    int next_phase = phase + 1;
#if (CONFIG_INITIAL_SWING_FOOT == 1)  
    if(phase == WkConfigPhase::lift_up) { next_phase = WkConfigPhase::double_contact_2; }
#endif
     printf("next phase: %i\n", next_phase);
    dynacore::Vect3 next_local_frame_location;

    if(phase == WkConfigPhase::double_contact_1) {
        //if(phase == WkConfigPhase::right_swing_start_trans) {
        ++num_step_;
        printf("%i th step:\n", num_step_);
        // printf("One swing done: Next Right Leg Swing\n");
        sp_->stance_foot_ = sagitP3_link::l_ankle;

        // Global Frame Update
        robot_sys_->getPos(sagitP3_link::l_ankle, next_local_frame_location);
        // when it start the left leg is already stance foot and 
        // global set by 0.15
        if(num_step_>1) { 
            sp_->global_pos_local_ += next_local_frame_location; 
        }
    }
    if(phase == WkConfigPhase::double_contact_2){
        //if(phase == WkConfigPhase::left_swing_start_trans){
        ++num_step_;
        printf("%i th step:\n", num_step_);

        sp_->stance_foot_ = sagitP3_link::r_ankle;

        // Global Frame Update
        robot_sys_->getPos(sagitP3_link::r_ankle, next_local_frame_location);
        sp_->global_pos_local_ += next_local_frame_location;
    }
    sp_->num_step_copy_ = num_step_;

    if(next_phase == WkConfigPhase::NUM_WALKING_PHASE) {
        return WkConfigPhase::double_contact_1;
    }
    else{ 
        return next_phase; 
    }
}

void WalkingConfigTest::_SettingParameter(){
    // Setting Parameters
    ParamHandler handler(SagitP3ConfigPath"TEST_walking.yaml");

    double tmp; bool b_tmp;
    std::vector<double> tmp_vec;
    std::string tmp_str;

    // Start Phase
    handler.getInteger("start_phase", phase_);

    //// Posture Setup
    // Initial JPos
    handler.getVector("initial_jpos", tmp_vec);
    ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);
    // CoM Height
    handler.getValue("body_height", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

    ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
    ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

    ((SwingPlanningCtrl*)right_swing_ctrl_)->setStanceHeight(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setStanceHeight(tmp);
    ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

    //// Timing Setup
    handler.getValue("jpos_initialization_time", tmp);
    ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
    handler.getValue("com_lifting_time", tmp);
    ((DoubleContactTransCtrl*)body_up_ctrl_)->setStanceTime(tmp);

    // Stance Time
    handler.getValue("stance_time", tmp);
    ((BodyCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyStanceTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyStanceTime(tmp);

    // Swing & prime Time
    handler.getValue("swing_time", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setSwingTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setSwingTime(tmp);

    // Transition Time
    handler.getValue("st_transition_time", tmp);
    ((SingleContactTransCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
    ((SingleContactTransCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

    ((SwingPlanningCtrl*)right_swing_ctrl_)->notifyTransitionTime(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->notifyTransitionTime(tmp);
    //// Planner Setup
    handler.getBoolean("replanning", b_tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setReplanning(b_tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setReplanning(b_tmp);

    handler.getValue("double_stance_mix_ratio", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setDoubleStanceRatio(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setDoubleStanceRatio(tmp);

    handler.getValue("transition_phase_mix_ratio", tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

    handler.getBoolean("contact_switch_check", b_tmp);
    ((SwingPlanningCtrl*)right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
    ((SwingPlanningCtrl*)left_swing_ctrl_)->setContactSwitchCheck(b_tmp);

    printf("[Walking Body Test] Complete to Setup Parameters\n");
}
