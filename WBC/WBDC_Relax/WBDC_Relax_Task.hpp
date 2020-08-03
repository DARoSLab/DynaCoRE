#ifndef WHOLE_BODY_DYNAMIC_CONTROL_RELAXED_TASK
#define WHOLE_BODY_DYNAMIC_CONTROL_RELAXED_TASK

#include <Task.hpp>

class WBDC_Relax_Task: public Task{
public:
  WBDC_Relax_Task(int dim);
  virtual ~WBDC_Relax_Task();

  int getRelaxed_Dim(){ return dim_relaxed_;}
  void setRelaxedOpCtrl(const std::vector<bool> & relaxed_op);
  void getSdelta(dynacore::Matrix & Sd){ Sd = S_del_; }
protected:
  virtual bool _AdditionalUpdate(){ return true;}
  int dim_relaxed_;
  dynacore::Matrix S_del_;
};

#endif
