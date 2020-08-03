#ifndef STATE_PROVIDER_Cheetah3
#define STATE_PROVIDER_Cheetah3

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace dynacore;

class RobotSystem;

class Cheetah3_StateProvider{
public:
  static Cheetah3_StateProvider* getStateProvider();
  ~Cheetah3_StateProvider(){}

  dynacore::Vect3 imu_ang_vel_;

  double curr_time_;

  Vector Q_;
  Vector Qdot_;
  Vector jpos_ini_;
  Vector des_jpos_prev_;
   
  dynacore::Vect3 global_pos_local_;
  dynacore::Vect2 des_location_;

  int b_fr_contact_;
  int b_fl_contact_;
  int b_hr_contact_;
  int b_hl_contact_;
  int num_step_copy_;

private:
  Cheetah3_StateProvider();
};


#endif
