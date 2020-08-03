#ifndef KINEMATICS_FOOT_TASK_SagitP3
#define KINEMATICS_FOOT_TASK_SagitP3

#include <WBLC/KinTask.hpp>

class SagitP3_StateProvider;
class RobotSystem;

// FootOri_{Rz}, Foot (x, y, z)
class FootTask: public KinTask{
public:
  FootTask(const RobotSystem*, int swing_foot);
  virtual ~FootTask();

protected:
  int swing_foot_;
  int stance_foot_;

  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true;}

  SagitP3_StateProvider* sp_;
  const RobotSystem* robot_sys_;
};

#endif
