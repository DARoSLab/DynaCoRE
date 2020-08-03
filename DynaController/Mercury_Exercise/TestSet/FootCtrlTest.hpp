#ifndef FOOT_CTRL_TEST
#define FOOT_CTRL_TEST

#include <Test.hpp>

class RobotSystem;

enum FOOT_TEST_PHASE{
  FOOT_TEST_INI= 0,
  FOOT_TEST_SWING = 1,
  NUM_FOOT_TEST_PHASE = 2
};

class FootCtrlTest: public Test{
public:
  FootCtrlTest(RobotSystem*);
  virtual ~FootCtrlTest();

  virtual void TestInitialization();
  void getTorqueInput(dynacore::Vector & gamma);

protected:
  int swing_foot_;
  void _ParameterCtrlSetting();
  virtual int _NextPhase(const int & phase);

  Controller* jpos_ini_;
  Controller* foot_ctrl_;
};

#endif
