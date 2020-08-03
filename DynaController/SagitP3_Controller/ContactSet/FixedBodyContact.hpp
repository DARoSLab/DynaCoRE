#ifndef SagitP3_FIXED_BODY_CONTACT
#define SagitP3_FIXED_BODY_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class SagitP3_StateProvider;

class FixedBodyContact: public WBDC_ContactSpec{
public:
  FixedBodyContact(RobotSystem* );
  virtual ~FixedBodyContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  RobotSystem* robot_sys_;
  SagitP3_StateProvider* sp_;
};

#endif
