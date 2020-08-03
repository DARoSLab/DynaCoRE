#include "WalkingBodyTest.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransBodyCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransCoMCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyRxRyRzCtrl.hpp>
#include <Mercury_Controller/CtrlSet/BodyFootJPosPlanningCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCtrl.hpp>
#include <Mercury_Controller/CtrlSet/TransitionCoMCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Planner/PIPM_FootPlacementPlanner/Reversal_LIPM_Planner.hpp>
#include <Utils/DataManager.hpp>

#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>


WalkingBodyTest::WalkingBodyTest(RobotSystem* robot):Test(robot),
                           num_step_(0)
{
  sp_ = Mercury_StateProvider::getStateProvider();
#if (INITIAL_SWING_FOOT == 0)
  sp_->stance_foot_ = mercury_link::leftFoot;
#else
  sp_->stance_foot_ = mercury_link::rightFoot;
#endif
  sp_->global_pos_local_[1] = 0.15;
  robot_sys_ = robot;
  reversal_planner_ = new Reversal_LIPM_Planner();
  phase_ = WkBodyPhase::initiation;

  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  body_up_ctrl_ = new ContactTransBodyCtrl(robot);
  body_fix_ctrl_ = new BodyRxRyRzCtrl(robot);
  // Right
  right_swing_start_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::rightFoot, false);
      //new TransitionCoMCtrl(robot, mercury_link::rightFoot, false);
  jpos_right_swing_ctrl_ = 
      new BodyFootJPosPlanningCtrl(robot, mercury_link::rightFoot, reversal_planner_);
  right_swing_end_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::rightFoot, true);
      //new TransitionCoMCtrl(robot, mercury_link::rightFoot, true);
  // Left
  left_swing_start_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::leftFoot, false);
      //new TransitionCoMCtrl(robot, mercury_link::leftFoot, false);
  jpos_left_swing_ctrl_ = 
      new BodyFootJPosPlanningCtrl(robot, mercury_link::leftFoot, reversal_planner_);
  left_swing_end_trans_ctrl_ = 
      new TransitionCtrl(robot, mercury_link::leftFoot, true);
      //new TransitionCoMCtrl(robot, mercury_link::leftFoot, true);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(body_up_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(right_swing_start_trans_ctrl_);
  state_list_.push_back(jpos_right_swing_ctrl_);
  state_list_.push_back(right_swing_end_trans_ctrl_);
  state_list_.push_back(body_fix_ctrl_);
  state_list_.push_back(left_swing_start_trans_ctrl_);
  state_list_.push_back(jpos_left_swing_ctrl_);
  state_list_.push_back(left_swing_end_trans_ctrl_);

  _SettingParameter();
  printf("[Walking Test] Constructed\n");
}

WalkingBodyTest::~WalkingBodyTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void WalkingBodyTest::TestInitialization(){
  // Planner
  reversal_planner_->PlannerInitialization(MercuryConfigPath"PLANNER_velocity_reversal");

  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  body_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  body_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");

  // Transition
  right_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
  right_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");
  left_swing_start_trans_ctrl_->CtrlInitialization("CTRL_trans");
  left_swing_end_trans_ctrl_->CtrlInitialization("CTRL_trans");

  // Swing
  jpos_right_swing_ctrl_->CtrlInitialization("CTRL_right_walking_swing");
  jpos_left_swing_ctrl_->CtrlInitialization("CTRL_left_walking_swing");
}

int WalkingBodyTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
#if (INITIAL_SWING_FOOT == 1)  
    if(phase == WkBodyPhase::lift_up) { next_phase = WkBodyPhase::double_contact_2; }
#endif
   //printf("next phase: %i\n", next_phase);
  
  if(phase == WkBodyPhase::double_contact_1) {
  //if(phase == WkBodyPhase::right_swing_start_trans) {
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
  if(phase == WkBodyPhase::double_contact_2){
  //if(phase == WkBodyPhase::left_swing_start_trans){
    ++num_step_;
    printf("%i th step:\n", num_step_);

    sp_->stance_foot_ = mercury_link::rightFoot;

    // Global Frame Update
    dynacore::Vect3 next_local_frame_location;
    robot_sys_->getPos(mercury_link::rightFoot, next_local_frame_location);
     //printf("One swing done: Next Left Leg Swing\n");
    //dynacore::pretty_print(sp_->global_pos_local_, std::cout, "global local");
    sp_->global_pos_local_ += next_local_frame_location;
    //dynacore::pretty_print(next_local_frame_location, std::cout, "next local frame");
    //dynacore::pretty_print(sp_->global_pos_local_, std::cout, "global local");
    sp_->global_foot_height_ = next_local_frame_location[2];
  }


  // TEST
  if (num_step_ > 50) {
      exit(0);
  }

  if(next_phase == WkBodyPhase::NUM_WALKING_PHASE) {
    return WkBodyPhase::double_contact_1;
  }
  else{ return next_phase; }
}

void WalkingBodyTest::_SettingParameter(){
  // Setting Parameters
  ParamHandler handler(MercuryConfigPath"TEST_walking_body.yaml");

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
  handler.getValue("com_height", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceHeight(tmp);
  ((BodyRxRyRzCtrl*)body_fix_ctrl_)->setStanceHeight(tmp);

  ((TransitionCtrl*)right_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)right_swing_end_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)left_swing_start_trans_ctrl_)->setStanceHeight(tmp);
  ((TransitionCtrl*)left_swing_end_trans_ctrl_)->setStanceHeight(tmp);

  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setStanceHeight(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setStanceHeight(tmp);
  ((Reversal_LIPM_Planner*)reversal_planner_)->setOmega(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransBodyCtrl*)body_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("stance_time", tmp);
  ((BodyRxRyRzCtrl*)body_fix_ctrl_)->setStanceTime(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->notifyStanceTime(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->notifyStanceTime(tmp);

  // Swing & prime Time
  handler.getValue("swing_time", tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setSwingTime(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setSwingTime(tmp);

  // Transition Time
  handler.getValue("st_transition_time", tmp);
  ((TransitionCtrl*)right_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)right_swing_end_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_start_trans_ctrl_)->setTransitionTime(tmp);
  ((TransitionCtrl*)left_swing_end_trans_ctrl_)->setTransitionTime(tmp);

  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->notifyTransitionTime(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->notifyTransitionTime(tmp);

  //// Planner Setup
  handler.getValue("planning_frequency", tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setPlanningFrequency(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setPlanningFrequency(tmp);

  handler.getValue("double_stance_mix_ratio", tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setDoubleStanceRatio(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setDoubleStanceRatio(tmp);


  handler.getValue("transition_phase_mix_ratio", tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setTransitionPhaseRatio(tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setTransitionPhaseRatio(tmp);

  handler.getBoolean("contact_switch_check", b_tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_right_swing_ctrl_)->setContactSwitchCheck(b_tmp);
  ((BodyFootJPosPlanningCtrl*)jpos_left_swing_ctrl_)->setContactSwitchCheck(b_tmp);


  printf("[Walking Body Test] Complete to Setup Parameters\n");
}
