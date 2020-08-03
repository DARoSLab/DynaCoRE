#ifndef TELLO_POINT_CONTACT
#define TELLO_POINT_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class TELLO_StateProvider;

class PointContact: public WBDC_ContactSpec{
public:
  PointContact(const RobotSystem* robot, int contact_pt);
  virtual ~PointContact();

  void setMaxFz(double max_fz){ max_Fz_ = max_fz; }

protected:
  double max_Fz_;
  int dim_U_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const RobotSystem* robot_sys_;
  TELLO_StateProvider* sp_;

  int contact_pt_;
};

#endif
