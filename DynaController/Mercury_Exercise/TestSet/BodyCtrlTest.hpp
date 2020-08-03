#ifndef BODY_CONTROL_TEST
#define BODY_CONTROL_TEST

#include <Test.hpp>

class RobotSystem;

enum BCPhase{
  initial_jpos = 0,
  lift_up = 1,
  stay_up = 2,
  body_ctrl = 3,
  NUM_BC_PHASE
};

class BodyCtrlTest: public Test{
public:
  BodyCtrlTest(RobotSystem* );
  virtual ~BodyCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_fix_ctrl_;
  Controller* body_shake_ctrl_;
};

#endif
