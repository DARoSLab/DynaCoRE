#include "WalkingConfigTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyJPosCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ConfigBodyFootPlanningCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionConfigCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

WalkingConfigTest::WalkingConfigTest(RobotSystem* robot):Test(robot),
                           num_step_(0)
{
  sp_ = Mercury_StateProvider::getStateProvider();
#if (CONFIG_INITIAL_SWING_FOOT == 0)
  sp_->stance_foot_ = mercury_link::leftFoot;
#else
  sp_->stance_foot_ = mercury_link::rightFoot;
#endif
  sp_->global_pos_local_[1] = 0.15;
  robot_sys_ = robot;
  reversal_planner_ = new Reversal_LIPM_Planner();
  phase_ = WkConfigPhase::initiation;

  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransBodyCtrl(robot);
  config_body_fix_ctrl_ = new BodyJPosCtrl(robot);
  // Right
  right_swing_start_trans_ctrl_ = 
      new TransitionConfigCtrl(robot, mercury_link::rightFoot, false);
  config_right_swing_ctrl_ = 
      new ConfigBodyFootPlanningCtrl(robot, mercury_link::rightFoot, reversal_planner_);
  right_swing_end_trans_ctrl_ = 
      new TransitionConfigCtrl(robot, mercury_link::rightFoot, true);
  // Left
  left_swing_start_trans_ctrl_ = 
      new TransitionConfigCtrl(robot, mercury_link::leftFoot, false);
  config_left_swing_ctrl_ = 
      new ConfigBodyFootPlanningCtrl(robot, mercury_link::leftFoot, reversal_planner_);
  left_swing_end_trans_ctrl_ = 
      new TransitionConfigCtrl(robot, mercury_link::leftFoot, true);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(config_body_fix_ctrl_);
  state_list_.push_back(right_swing_start_trans_ctrl_);
  state_list_.push_back(config_right_swing_ctrl_);
  state_list_.push_back(right_swing_end_trans_ctrl_);
  state_list_.push_back(config_body_fix_ctrl_);
  state_list_.push_back(left_swing_start_trans_ctrl_);
  state_list_.push_back(config_left_swing_ctrl_);
  state_list_.push_back(left_swing_end_trans_ctrl_);

  DataManager::GetDataManager()->RegisterData(
          &(((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->curr_foot_pos_des_), 
          VECT3, "rfoot_pos_des", 3);
  DataManager::GetDataManager()->RegisterData(
          &(((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->curr_foot_pos_des_), 
          VECT3, "lfoot_pos_des", 3);

  DataManager::GetDataManager()->RegisterData(
          &(((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->curr_foot_vel_des_), 
          VECT3, "rfoot_vel_des", 3);
  DataManager::GetDataManager()->RegisterData(
          &(((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->curr_foot_vel_des_), 
          VECT3, "lfoot_vel_des", 3);


  _SettingParameter();
  printf("[Walking Config Test] Constructed\n");
}

WalkingConfigTest::~WalkingConfigTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void WalkingConfigTest::TestInitialization(){
  // Planner
  reversal_planner_->PlannerInitialization(MercuryConfigPath"PLANNER_velocity_reversal");

  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  config_body_fix_ctrl_->CtrlInitialization("CTRL_fix_config");

  // Transition
  right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
  right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
  left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_config_trans");
  left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_config_trans");

  // Swing
  config_right_swing_ctrl_->CtrlInitialization("CTRL_config_right_walking_swing");
  config_left_swing_ctrl_->CtrlInitialization("CTRL_config_left_walking_swing");
}

int WalkingConfigTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
#if (CONFIG_INITIAL_SWING_FOOT == 1)  
    if(phase == WkConfigPhase::lift_up) { next_phase = WkConfigPhase::double_contact_2; }
#endif
   // printf("next phase: %i\n", next_phase);
  
  if(phase == WkConfigPhase::double_contact_1) {
  //if(phase == WkConfigPhase::right_swing_start_trans) {
    ++num_step_;
    printf("%i th step:\n", num_step_);
    // printf("One swing done: Next Right Leg Swing\n");
    sp_->stance_foot_ = mercury_link::leftFoot;

    // Global Frame Update
    dynacore::Vect3 next_local_frame_location;
    robot_sys_->getPos(mercury_link::leftFoot, next_local_frame_location);
    // when it start the left leg is already stance foot and 
    // global set by 0.15
    if(num_step_>1) sp_->global_pos_local_ += next_local_frame_location;
    sp_->global_foot_height_ = next_local_frame_location[2];
  }
  if(phase == WkConfigPhase::double_contact_2){
  //if(phase == WkConfigPhase::left_swing_start_trans){
    ++num_step_;
    printf("%i th step:\n", num_step_);

    sp_->stance_foot_ = mercury_link::rightFoot;

    // Global Frame Update
    dynacore::Vect3 next_local_frame_location;
    robot_sys_->getPos(mercury_link::rightFoot, next_local_frame_location);
    sp_->global_pos_local_ += next_local_frame_location;
    sp_->global_foot_height_ = next_local_frame_location[2];
  }

  // TEST
  if (num_step_ > 10) {
      exit(0);
  }

  if(next_phase == WkConfigPhase::NUM_WALKING_PHASE) {
    return WkConfigPhase::double_contact_1;
  }
  else{ return next_phase; }
}

void WalkingConfigTest::_SettingParameter(){
  // Setting Parameters
  ParamHandler handler(MercuryConfigPath"TEST_walking_config.yaml");

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
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((BodyJPosCtrl*)config_body_fix_ctrl_)->setStanceHeight(tmp);

  ((TransitionConfigCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionConfigCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionConfigCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionConfigCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setStanceHeight(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setStanceHeight(tmp);
  ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("stance_time", tmp);
  ((BodyJPosCtrl*)config_body_fix_ctrl_)->setStanceTime(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->notifyStanceTime(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->notifyStanceTime(tmp);

  // Swing & prime Time
  handler.getValue("swing_time", tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setSwingTime(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setSwingTime(tmp);

  // Transition Time
  handler.getValue("st_transition_time", tmp);
  ((TransitionConfigCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionConfigCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionConfigCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionConfigCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->notifyTransitionTime(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->notifyTransitionTime(tmp);

  //// Planner Setup
  handler.getValue("planning_frequency", tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setPlanningFrequency(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setPlanningFrequency(tmp);

  handler.getValue("double_stance_mix_ratio", tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setDoubleStanceRatio(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setDoubleStanceRatio(tmp);


  handler.getValue("transition_phase_mix_ratio", tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

  handler.getBoolean("contact_switch_check", b_tmp);
  ((ConfigBodyFootPlanningCtrl*)config_right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
  ((ConfigBodyFootPlanningCtrl*)config_left_swing_ctrl_)->setContactSwitchCheck(b_tmp);


  printf("[Walking Body Test] Complete to Setup Parameters\n");
}
