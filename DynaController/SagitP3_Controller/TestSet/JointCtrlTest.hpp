#ifndef JOINT_POSITION_TEST_SagitP3
#define JOINT_POSITION_TEST_SagitP3

#include <Test.hpp>

class RobotSystem;

enum JPOS_TEST_PHASE{
  JPOS_TEST_INI = 0,
  JPOS_TEST_SWING = 1,
  NUM_JPOS_TEST = 2
};

class JointCtrlTest: public Test{
public:
  JointCtrlTest(RobotSystem* );
  virtual ~JointCtrlTest();

  virtual void TestInitialization();

protected:
  void _ParameterSetting();
  virtual int _NextPhase(const int & phase);

  Controller* jpos_ctrl_ini_;
  Controller* jpos_ctrl_;
};

#endif
