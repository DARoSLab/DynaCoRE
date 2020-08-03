#ifndef TRANSITION_CONFIGURATION_CONTROLLER
#define TRANSITION_CONFIGURATION_CONTROLLER

#include <Controller.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

class RobotSystem;
class Mercury_StateProvider;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;
class WBWC;

class TransitionConfigCtrl: public Controller{
public:
  TransitionConfigCtrl(RobotSystem* robot, int moving_foot, bool b_increase);
  virtual ~TransitionConfigCtrl();

  virtual void OneStep(void* _cmd);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setTransitionTime(double time){ end_time_ = time; }
  void setStanceHeight(double height) {
    des_body_height_ = height;
    b_set_height_target_ = true;
  }

protected:

  bool b_set_height_target_;
  double des_body_height_;

  double end_time_;
  int moving_foot_;
  bool b_increase_; // Increasing or decreasing reaction force
  double max_rf_z_;
  double min_rf_z_;
 
  WBWC* wbwc_;
  Task* config_task_;
  WBDC_ContactSpec* double_contact_;
  WBDC_Rotor* wbdc_rotor_;
  WBDC_Rotor_ExtraData* wbdc_rotor_data_;


  dynacore::Vector des_jpos_;
  dynacore::Vector des_jvel_;
  dynacore::Vector des_jacc_;
  
  dynacore::Vector body_pos_ini_;
  dynacore::Vect3 ini_body_pos_;


  void _body_task_setup();
  void _double_contact_setup();
  void _body_ctrl_wbdc_rotor(dynacore::Vector & gamma);

  Mercury_StateProvider* sp_;
  Mercury_InvKinematics inv_kin_;
  double ctrl_start_time_;
};
#endif
