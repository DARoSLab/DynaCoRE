#ifndef WBDC_BODY_HEIGHT_ORIENTATION_TASK
#define WBDC_BODY_HEIGHT_ORIENTATION_TASK

#include <Task.hpp>

class Mercury_StateProvider;

class BodyOriTask: public Task{
public:
  BodyOriTask(); // X, Y, Z, Rx, Ry, Rz
  virtual ~BodyOriTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

protected:
  // Update op_cmd_
  virtual bool _UpdateCommand(void* pos_des,
                              const dynacore::Vector & vel_des,
                              const dynacore::Vector & acc_des);
  // Update Jt_
  virtual bool _UpdateTaskJacobian();
  // Update JtDotQdot_
  virtual bool _UpdateTaskJDotQdot();
  virtual bool _AdditionalUpdate(){ return true;}

  Mercury_StateProvider* sp_;
};

#endif
