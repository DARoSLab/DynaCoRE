#ifndef KINEMATICS_BODY_TASK_SagitP3
#define KINEMATICS_BODY_TASK_SagitP3
// Task consist of virtual joint (6)
// (X, Y, Z), (Rx, Ry, Rz)
#include <WBLC/KinTask.hpp>

class SagitP3_StateProvider;
class RobotSystem;

class BodyTask: public KinTask{
public:
  BodyTask(const RobotSystem*);
  virtual ~BodyTask();

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true; }

  const RobotSystem* robot_sys_;
  SagitP3_StateProvider* sp_;
};

#endif
