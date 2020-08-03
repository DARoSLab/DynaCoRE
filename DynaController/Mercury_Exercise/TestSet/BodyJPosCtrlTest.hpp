#ifndef BODY_JPOS_CONTROL_TEST
#define BODY_JPOS_CONTROL_TEST

#include <Test.hpp>

class RobotSystem;

enum BCJPosPhase{
  BCJPOS_initial_jpos = 0,
  BCJPOS_lift_up = 1,
  BCJPOS_body_ctrl = 2,
  NUM_BCJPOS_PHASE
};

class BodyJPosCtrlTest: public Test{
public:
  BodyJPosCtrlTest(RobotSystem* );
  virtual ~BodyJPosCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller* jpos_ctrl_;
  Controller* body_up_ctrl_;
  Controller* body_jpos_ctrl_;

};

#endif
