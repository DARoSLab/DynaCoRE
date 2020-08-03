#ifndef STATE_PROVIDER_TELLO
#define STATE_PROVIDER_TELLO

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace dynacore;

class RobotSystem;

class TELLO_StateProvider{
public:
  static TELLO_StateProvider* getStateProvider();
  ~TELLO_StateProvider(){}

  dynacore::Vect3 imu_ang_vel_;
  // Important!!!!!!!!
  int stance_foot_;

  double curr_time_;

  Vector Q_;
  Vector Qdot_;
  Vector jpos_ini_;
  Vector des_jpos_prev_;
   
  dynacore::Vect3 global_pos_local_;
  dynacore::Vect2 des_location_;

  int b_rfoot_contact_;
  int b_lfoot_contact_;
  int num_step_copy_;

private:
  TELLO_StateProvider();
};


#endif
