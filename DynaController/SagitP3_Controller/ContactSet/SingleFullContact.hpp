#ifndef SagitP3_SINGLE_Full_CONTACT
#define SagitP3_SINGLE_Full_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class SagitP3_StateProvider;

class SingleFullContact: public WBDC_ContactSpec{
public:
  SingleFullContact(const RobotSystem* robot, int contact_pt);
  virtual ~SingleFullContact();

  void setMaxFz(double max_fz){ max_Fz_ = max_fz; }

protected:
  double max_Fz_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const RobotSystem* robot_sys_;
  SagitP3_StateProvider* sp_;

  void _setU(double x, double y, double mu, dynacore::Matrix & U);
  
  int contact_pt_;
  int dim_U_;
};

#endif
