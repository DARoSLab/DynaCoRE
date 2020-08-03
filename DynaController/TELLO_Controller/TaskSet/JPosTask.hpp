#ifndef JPOS_TASK_TELLO
#define JPOS_TASK_TELLO

#include <WBLC/KinTask.hpp>

class TELLO_StateProvider;

class JPosTask: public KinTask{
public:
  JPosTask();
  virtual ~JPosTask();

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

  TELLO_StateProvider* sp_;
};

#endif
