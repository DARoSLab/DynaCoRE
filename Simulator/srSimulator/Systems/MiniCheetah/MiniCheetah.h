#ifndef MiniCheetah_H
#define MiniCheetah_H

#include <srSysGenerator/SystemGenerator.h>

class MiniCheetah: public SystemGenerator {
 public:
  MiniCheetah();
  virtual ~MiniCheetah();

 private:
  virtual void _SetCollision();
  virtual void _SetInitialConf();
  virtual void _SetJointLimit();

  std::vector<srCollision*> collision_;
};

#endif
