#ifndef WALKING_JOINT_POSITION_TEST
#define WALKING_JOINT_POSITION_TEST

#include <Test.hpp>
class Mercury_StateProvider;
class Planner;
class Mercury_InvKinematics;

#define CONFIG_INITIAL_SWING_FOOT 0 // 0: right, 1: left
#define SWING_CTRL_TYPE 1 // 0: hierarchy, 1: normal, 2: JPos, 3: JPos Traj

namespace WkJPosPhase{
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

class WalkingJPosTest: public Test{
public:
  WalkingJPosTest(RobotSystem*);
  virtual ~WalkingJPosTest();
  virtual void TestInitialization();

protected:
  int num_step_;
  Mercury_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  void _SetupStancePosture(double height);

  dynacore::Vector stance_jpos_;
  dynacore::Vector swing_leg_mid_jpos_;

  Planner* reversal_planner_;

  Controller* jpos_ctrl_;
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

  Mercury_InvKinematics* inv_kin_;
};
#endif
