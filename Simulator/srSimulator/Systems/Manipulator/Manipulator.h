#ifndef Manipulator_H
#define Manipulator_H

#include <srSysGenerator/SystemGenerator.h>

class Manipulator: public SystemGenerator {
 public:
  Manipulator();
  virtual ~Manipulator();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif
