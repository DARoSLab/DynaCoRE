#ifndef COM_STANCE_SWING_TEST
#define COM_STANCE_SWING_TEST

#include <Test.hpp>
#include <ParamHandler/ParamHandler.hpp>
class Mercury_StateProvider;

namespace CoMStanceSwingPhase{
  constexpr int stance_swing_initiation = 0;
  constexpr int stance_swing_lift_up = 1;
  constexpr int stance_swing_double_contact_1 = 2;
  constexpr int stance_swing_swing_start_trans = 3;
  constexpr int stance_swing_swing = 4;
  constexpr int NUM_STANCE_SWING_PHASE = 5;
};

class CoMStanceSwingTest: public Test{
public:
  CoMStanceSwingTest(RobotSystem*);
  virtual ~CoMStanceSwingTest();
  virtual void TestInitialization();

protected:
  int swing_foot_;

  Mercury_StateProvider* sp_;
  virtual int _NextPhase(const int & phase);
  void _SettingParameter(ParamHandler & );

  Controller* jpos_ctrl_;
  Controller* com_up_ctrl_;
  Controller* com_fix_ctrl_;
  Controller* swing_start_trans_ctrl_;
  Controller* swing_ctrl_;
  Controller* jpos_swing_ctrl_;
  
  const RobotSystem* robot_sys_;
};
#endif
