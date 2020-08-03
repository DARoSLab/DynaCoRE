#ifndef WALKING_TEST_ATLAS
#define WALKING_TEST_ATLAS

#include <Test.hpp>
class Atlas_StateProvider;
class Planner;

namespace WkPhase{
  constexpr int lift_up = 0;
  constexpr int double_contact_1 = 1;
  constexpr int right_swing_start_trans = 2;
  constexpr int right_swing = 3;
  constexpr int right_swing_end_trans = 4;
  constexpr int double_contact_2 = 5;
  constexpr int left_swing_start_trans = 6;
  constexpr int left_swing = 7;
  constexpr int left_swing_end_trans = 8;
  constexpr int NUM_WALKING_PHASE = 9;
};

class WalkingTest: public Test{
public:
  WalkingTest(RobotSystem*);
  virtual ~WalkingTest();
  virtual void TestInitialization();

protected:
  int num_step_;
  Atlas_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();

  Planner* reversal_planner_;

  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  // Right
  Controller* right_swing_start_trans_ctrl_;
  Controller* right_swing_ctrl_;
  Controller* right_swing_end_trans_ctrl_;
  // Left
  Controller* left_swing_start_trans_ctrl_;
  Controller* left_swing_ctrl_;
  Controller* left_swing_end_trans_ctrl_;
  const RobotSystem* robot_sys_;
};
#endif
