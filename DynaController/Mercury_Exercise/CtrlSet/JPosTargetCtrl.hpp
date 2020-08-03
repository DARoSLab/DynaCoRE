#ifndef JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL
#define JOINT_POSITION_MOVE_TO_TARGET_POS_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_Relax;
class WBDC_Relax_ExtraData;
class WBDC_Relax_Task;
class WBDC_ContactSpec;

class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class JPosTargetCtrl: public Controller{
public:
  JPosTargetCtrl(RobotSystem* );
  virtual ~JPosTargetCtrl();

  virtual void OneStep(dynacore::Vector & gamma);
  virtual void FirstVisit();
  virtual void LastVisit();
  virtual bool EndOfPhase();

  virtual void CtrlInitialization(const std::string & setting_file_name);

  void setMovingTime(double time) { end_time_ = time; }
  void setTargetPosition(const std::vector<double> & jpos);
protected:
  double end_time_;

  WBDC_Relax* wbdc_;
  WBDC_Relax_ExtraData* wbdc_data_;
  WBDC_Relax_Task* jpos_task_;
  WBDC_ContactSpec* fixed_body_contact_;

  WBDC_Rotor* wbdc_rotor_;
  WBDC_Rotor_ExtraData* wbdc_rotor_data_;
 
  dynacore::Vector jpos_ini_;
  dynacore::Vector jpos_target_;
  
  void _jpos_task_setup();
  void _fixed_body_contact_setup();
  void _jpos_ctrl(dynacore::Vector & gamma);
  void _jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma);

  Mercury_StateProvider* sp_;
  double ctrl_start_time_;
};

#endif
