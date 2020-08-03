#ifndef HUMANOID_H
#define HUMANOID_H

#include <srSysGenerator/SystemGenerator.h>

class Humanoid: public SystemGenerator {
 public:
  Humanoid();
  virtual ~Humanoid();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  void _HumanoidInitial();
  void _TelloInitial();
  std::vector<srCollision*> collision_;
};

#endif
