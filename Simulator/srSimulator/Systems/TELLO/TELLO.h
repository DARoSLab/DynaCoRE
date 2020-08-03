#ifndef TELLO_H
#define TELLO_H

#include <srSysGenerator/SystemGenerator.h>

class TELLO: public SystemGenerator {
 public:
  TELLO();
  virtual ~TELLO();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  void _setTELLO_InitialConfig();
  void _setHumanoid_InitialConfig();

  std::vector<srCollision*> collision_;
};

#endif
