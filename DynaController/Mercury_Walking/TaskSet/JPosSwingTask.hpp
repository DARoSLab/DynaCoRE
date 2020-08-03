#ifndef WBDC_Joint_Position_Swing_LEG_TASK
#define WBDC_Joint_Position_Swing_LEG_TASK
// swing leg joint (3)
#include <Task.hpp>

class Mercury_StateProvider;

class JPosSwingTask: public Task{
public:
  JPosSwingTask(int swing_leg);
  virtual ~JPosSwingTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

protected:
  int swing_leg_;
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true; }

  Mercury_StateProvider* sp_;
  int swing_leg_jidx_;
};

#endif
