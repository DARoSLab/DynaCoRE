#ifndef WBDC_COM_BODY_ORIENTATION_SWING_FOOT_JPOS_TASK
#define WBDC_COM_BODY_ORIENTATION_SWING_FOOT_JPOS_TASK

#include <Task.hpp>

class Mercury_StateProvider;
class RobotSystem;

// CoM_{x, y, z}, BodyOri_{Rx, Ry, Rz}, Foot (x, y, z)
class CoMFootJPosTask: public Task{
public:
  CoMFootJPosTask(RobotSystem*, int swing_foot);
  virtual ~CoMFootJPosTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

protected:
  int swing_foot_;

  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true;}

  int swing_leg_jidx_;
  const Mercury_StateProvider* sp_;
  const RobotSystem* robot_sys_;
};

#endif
