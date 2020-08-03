#ifndef Single_Stepping_CONFIGURATION_TEST_SagitP3
#define Single_Stepping_CONFIGURATION_TEST_SagitP3

#include <Test.hpp>
class SagitP3_StateProvider;

namespace SSteppingPhase{
  constexpr int lift_up = 0;
  constexpr int double_contact_1 = 1;
  constexpr int swing_start_trans = 2;
  constexpr int swing = 3;
  constexpr int swing_end_trans = 4;
  constexpr int NUM_SingleStepping_PHASE = 5;
};

class SingleSteppingTest: public Test{
public:
  SingleSteppingTest(RobotSystem*);
  virtual ~SingleSteppingTest();
  virtual void TestInitialization();

protected:
  int contact_pt_;
  int swing_pt_;
  int num_step_;

  SagitP3_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();

  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  
  Controller* swing_start_trans_ctrl_;
  Controller* swing_ctrl_;
  Controller* swing_end_trans_ctrl_;

  const RobotSystem* robot_sys_;
};
#endif
