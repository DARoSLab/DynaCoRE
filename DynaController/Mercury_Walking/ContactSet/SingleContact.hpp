#ifndef MERCURY_SINGLE_CONTACT
#define MERCURY_SINGLE_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class Mercury_StateProvider;

class SingleContact: public WBDC_ContactSpec{
public:
  SingleContact(const RobotSystem* robot, int contact_pt);
  virtual ~SingleContact();

protected:
  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const RobotSystem* robot_sys_;
  Mercury_StateProvider* sp_;

  int contact_pt_;
};

#endif
