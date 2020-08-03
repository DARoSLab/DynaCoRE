#ifndef WBDC_STANCE_LEG_TASK
#define WBDC_STANCE_LEG_TASK
// Task consist of virtual joint (6) and stance leg joint (3)
#include <Task.hpp>

class Mercury_StateProvider;

class StanceTask: public Task{
public:
  StanceTask(int stance_leg);
  virtual ~StanceTask();

  dynacore::Vector Kp_vec_;
  dynacore::Vector Kd_vec_;

protected:
  int stance_leg_;
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
  int stance_leg_jidx_;
};

#endif
