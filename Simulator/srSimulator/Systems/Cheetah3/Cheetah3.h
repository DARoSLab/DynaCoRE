#ifndef Cheetah3_H
#define Cheetah3_H

#include <srSysGenerator/SystemGenerator.h>

class Cheetah3: public SystemGenerator {
 public:
  Cheetah3();
  virtual ~Cheetah3();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif
