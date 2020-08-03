#ifndef STATE_PROVIDER_MERCURY
#define STATE_PROVIDER_MERCURY

#include <Utils/utilities.hpp>
#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>

using namespace dynacore;

class RobotSystem;

class Mercury_StateProvider{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static Mercury_StateProvider* getStateProvider();
  ~Mercury_StateProvider(){}

  double des_body_pitch_; // (rad)
    // Walking related data
  dynacore::Vect3 default_rfoot_loc_;
  dynacore::Vect3 default_lfoot_loc_;
  dynacore::Vector curr_jpos_des_;
  double Kp_roll_, Kp_pitch_;
  double Kd_roll_, Kd_pitch_;

  RobotSystem* jjpos_robot_sys_;
  dynacore::Vector mjpos_;
  dynacore::Vector jjpos_config_;
  dynacore::Vector jjvel_qdot_;
  dynacore::Vect3 jjpos_body_pos_;
  dynacore::Vect3 jjvel_body_vel_;
  dynacore::Vect3 jjpos_rfoot_pos_;
  dynacore::Vect3 jjpos_lfoot_pos_;

  dynacore::Quaternion body_ori_;
  dynacore::Vect3 body_ori_rpy_;
  dynacore::Quaternion body_ori_des_;
  dynacore::Vect3 body_ang_vel_;
  dynacore::Vect3 body_ang_vel_des_;
  dynacore::Vect3 imu_acc_inc_;
  dynacore::Vect3 imu_ang_vel_;
  dynacore::Vect3 imu_acc_;
  // Important!!!!!!!!
  int stance_foot_;

  double first_LED_x_;
  double first_LED_y_;

  bool initialized_;
  double curr_time_;
  int system_count_;

  Vector Q_;
  Vector Qdot_;
  Vector curr_torque_;

  
  dynacore::Vector reaction_forces_;
  dynacore::Vector qddot_cmd_;
  dynacore::Vector reflected_reaction_force_;

  dynacore::Vect3 global_pos_local_;
  dynacore::Vect3 global_jjpos_local_;
  
  double global_foot_height_;
  dynacore::Vect2 des_location_;

  dynacore::Vect3 CoM_pos_;
  dynacore::Vect3 CoM_vel_;
  dynacore::Vect2 est_CoM_vel_;
  
  dynacore::Vect2 est_mocap_body_vel_;
  dynacore::Vect3 est_mocap_body_pos_;

  dynacore::Vect3 com_pos_des_;
  dynacore::Vect3 com_vel_des_;

  dynacore::Vect3 ekf_body_pos_;
  dynacore::Vect3 ekf_body_vel_;

  dynacore::Vect3 body_pos_;
  dynacore::Vect3 body_vel_;
  dynacore::Vect3 body_pos_des_;
  dynacore::Vect3 body_vel_des_;

  dynacore::Vect2 average_vel_;

  dynacore::Vector jpos_des_;
  dynacore::Vector jvel_des_;
  dynacore::Vector jacc_des_;

  dynacore::Vector rotor_inertia_;

  void SaveCurrentData();

  int b_rfoot_contact_;
  int b_lfoot_contact_;
  // (x, y, x_dot, y_dot)
  dynacore::Vector estimated_com_state_;
  dynacore::Vector com_state_imu_; // (x, y, xdot, ydot, xddot, yddot)
  
  dynacore::Vect3 Rfoot_pos_;
  dynacore::Vect3 Rfoot_vel_;
  dynacore::Vect3 Lfoot_pos_;
  dynacore::Vect3 Lfoot_vel_;

  dynacore::Vect3 sim_imu_pos;
  dynacore::Vect3 sim_imu_vel;

  int phase_copy_;
  int num_step_copy_;

  dynacore::Vector led_kin_data_;

  dynacore::Vector filtered_jvel_;
  dynacore::Vect3 filtered_ang_vel_;

  // Test minimum jerk
  dynacore::Vect3 test_minjerk_pos;
  dynacore::Vect3 test_minjerk_vel;
  dynacore::Vect3 test_minjerk_acc;        

private:
  Mercury_StateProvider();
};


#endif
