#ifndef WALKING_TEST
#define WALKING_TEST

#include <Test.hpp>
class Mercury_StateProvider;
class Planner;

enum WKPhase{
  wk_initiation = 0,
  wk_lift_up = 1,
  wk_double_contact_1 = 2,
  wk_right_swing_start_trans = 3,
  wk_right_swing = 4,
  wk_right_swing_end_trans = 5,
  wk_double_contact_2 = 6,
  wk_left_swing_start_trans = 7,
  wk_left_swing = 8,
  wk_left_swing_end_trans = 9,
  NUM_WALKING_PHASE
};

class WalkingTest: public Test{
public:
  WalkingTest(RobotSystem*);
  virtual ~WalkingTest();
  virtual void TestInitialization();

protected:
  int num_step_;
  Mercury_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();

  Planner* reversal_planner_;

  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  // Right
  Controller* right_swing_start_trans_ctrl_;
  Controller* right_swing_ctrl_;
  Controller* jpos_right_swing_ctrl_;
  Controller* right_swing_end_trans_ctrl_;
  // Left
  Controller* left_swing_start_trans_ctrl_;
  Controller* left_swing_ctrl_;
  Controller* jpos_left_swing_ctrl_;
  Controller* left_swing_end_trans_ctrl_;
  const RobotSystem* robot_sys_;
};
#endif
