#ifndef COM_CONTROL_TEST
#define COM_CONTROL_TEST

#include <Test.hpp>

class RobotSystem;

namespace CoMCtrlPhase{
  constexpr int initial_jpos = 0;
  constexpr int lift_up = 1;
  constexpr int stay_up = 2;
  constexpr int body_ctrl = 3;
  constexpr int NUM_CoMCtrl_PHASE = 4;
};

class CoMCtrlTest: public Test{
public:
  CoMCtrlTest(RobotSystem* );
  virtual ~CoMCtrlTest();
  virtual void TestInitialization();

protected:
  virtual int _NextPhase(const int & phase);
  void _SettingParameter();
  
  Controller* jpos_ctrl_;
  Controller* com_up_ctrl_;
  Controller* com_fix_ctrl_;
  Controller* com_shake_ctrl_;
};

#endif
