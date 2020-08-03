#ifndef Humanoid_PITCH_CONTACT
#define Humanoid_PITCH_CONTACT

#include <WBDC/WBDC_ContactSpec.hpp>
class RobotSystem;
class Humanoid_StateProvider;

class PitchContact: public WBDC_ContactSpec{
public:
  PitchContact(const RobotSystem* robot, int contact_pt);
  virtual ~PitchContact();

  void setMaxFz(double max_fz){ max_Fz_ = max_fz; }
protected:
  double max_Fz_;
  int dim_U_;

  virtual bool _UpdateJc();
  virtual bool _UpdateJcDotQdot();
  virtual bool _UpdateUf();
  virtual bool _UpdateInequalityVector();

  const RobotSystem* robot_sys_;
  Humanoid_StateProvider* sp_;

  int contact_pt_;
};

#endif
