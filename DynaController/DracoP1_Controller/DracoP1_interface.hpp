#ifndef INTERFACE_H
#define INTERFACE_H

class Test;
class RobotSystem2D;

class interface{
public:
  interface():count_(0), running_time_(0.){}
  virtual ~interface(){}

  virtual void GetCommand(void* sensor_data,  std::vector<double> & command) = 0;

protected:
  Test* test_;
  RobotSystem2D* robot_sys_;

  int count_;
  double running_time_;
};

#endif
