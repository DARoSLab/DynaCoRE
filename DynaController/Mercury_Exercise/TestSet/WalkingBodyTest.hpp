#ifndef WALKING_BODY_TEST
#define WALKING_BODY_TEST

#include <Test.hpp>
class Mercury_StateProvider;
class Planner;

#define INITIAL_SWING_FOOT 0 // 0: right, 1: left

namespace WkBodyPhase{
  constexpr int initiation = 0;
  constexpr int lift_up = 1;
  constexpr int double_contact_1 = 2;
  constexpr int right_swing_start_trans = 3;
  constexpr int right_swing = 4;
  constexpr int right_swing_end_trans = 5;
  constexpr int double_contact_2 = 6;
  constexpr int left_swing_start_trans = 7;
  constexpr int left_swing = 8;
  constexpr int left_swing_end_trans = 9;
  constexpr int NUM_WALKING_PHASE = 10;
};

class WalkingBodyTest: public Test{
public:
  WalkingBodyTest(RobotSystem*);
  virtual ~WalkingBodyTest();
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
  Controller* jpos_right_swing_ctrl_;
  Controller* right_swing_end_trans_ctrl_;
  // Left
  Controller* left_swing_start_trans_ctrl_;
  Controller* jpos_left_swing_ctrl_;
  Controller* left_swing_end_trans_ctrl_;
  const RobotSystem* robot_sys_;
};
#endif
