#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>

BasicAccumulation::BasicAccumulation():OriEstimator(), com_state_(6){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  com_state_.setZero();

  count = 0;
  calibration_time = 2.5; // Seconds
  reset_once = false;

  // Bias Filter 
  bias_lp_frequency_cutoff = 2.0*3.1415*1.0; // 1Hz // (2*pi*frequency) rads/s 
  x_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  y_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  z_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);  
  x_acc_bias = 0.0;
  y_acc_bias = 0.0;  
  z_acc_bias = 0.0;

  //
  lp_frequency_cutoff = 2.0*3.1415*100; // 100Hz // (2*pi*frequency) rads/s
  x_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  y_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  z_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);

  gravity_mag = 9.81; // m/s^2;
  theta_x = 0.0;
}
BasicAccumulation::~BasicAccumulation(){}

void BasicAccumulation::CoMStateInitialization(
        const dynacore::Vect3 & com_pos, 
        const dynacore::Vect3 & com_vel){

    // com_state_[0] = com_pos[0];
    // com_state_[1] = com_pos[1];
    // com_state_[2] = com_vel[0];
    // com_state_[3] = com_vel[1];
}

void BasicAccumulation::getEstimatedCoMState(dynacore::Vector & com_state){
    com_state = com_state_;
}


void BasicAccumulation::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
                                                const std::vector<double> & acc,
                                                const std::vector<double> & ang_vel){
  global_ori_.w() = 1.;
  global_ori_.x() = 0.;
  global_ori_.y() = 0.;
  global_ori_.z() = 0.;

  for(int i(0); i<3; ++i){
      global_ang_vel_[i] = ang_vel[i];
    ini_acc_[i] = acc[i];
    }
}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & acc_inc,
                                      const std::vector<double> & ang_vel){
  // Orientation
  dynacore::Quaternion delt_quat;
  dynacore::Vect3 delta_th;
  double theta(0.);
  for(int i(0); i<3; ++i){
    delta_th[i] = ang_vel[i] * mercury::servo_rate;
    theta += delta_th[i] * delta_th[i];
  }

  if(fabs(theta) > 1.e-20){
    delt_quat.w() = cos(theta/2.);
    delt_quat.x() = sin(theta/2.) * delta_th[0]/theta;
    delt_quat.y() = sin(theta/2.) * delta_th[1]/theta;
    delt_quat.z() = sin(theta/2.) * delta_th[2]/theta;
  } else {
    delt_quat.w() = 1.;
    delt_quat.x() = 0.;
    delt_quat.y() = 0.;
    delt_quat.z() = 0.;
  }

  global_ori_ = dynacore::QuatMultiply(global_ori_, delt_quat);
  static int count(0);
  ++count;
  if(count%500 == 501){
    dynacore::pretty_print(acc, "[estimator] acc");
    dynacore::pretty_print(ang_vel, "[estimator] ang vel");

    dynacore::pretty_print(delt_quat, std::cout, "delta quat");
    dynacore::pretty_print(global_ori_, std::cout, "global ori");
  }
  dynacore::Quaternion ang_quat;
  ang_quat.w() = 0.;
  ang_quat.x() = ang_vel[0];
  ang_quat.y() = ang_vel[1];
  ang_quat.z() = ang_vel[2];

  dynacore::Quaternion quat_dot = dynacore::QuatMultiply(global_ori_, ang_quat, false);
  quat_dot = dynacore::QuatMultiply(quat_dot, global_ori_.inverse(), false);

  global_ang_vel_[0] = quat_dot.x();
  global_ang_vel_[1] = quat_dot.y();
  global_ang_vel_[2] = quat_dot.z();


  dynacore::Quaternion global_acc;
  global_acc.w() = 0.;
  global_acc.x() = acc[0];
  global_acc.y() = acc[1];
  global_acc.z() = acc[2];

  dynacore::Quaternion quat_acc = dynacore::QuatMultiply(global_ori_, global_acc, false);
    quat_acc = dynacore::QuatMultiply(quat_acc, global_ori_.inverse(), false);

    // com_state_[4] = quat_acc.x() - ini_acc_[0]; 
    // com_state_[5] = quat_acc.y() - ini_acc_[1];

    // Reset filters and velocities once after bias calibration time
    if (((count*mercury::servo_rate) > calibration_time) && (!reset_once)){
      // Reset the filters
      x_acc_low_pass_filter->clear();
      y_acc_low_pass_filter->clear();
      z_acc_low_pass_filter->clear();      
      reset_once = true;

      // Reset local velocities
      com_state_[2] = 0.0;
      com_state_[3] = 0.0;     

      // Reset local positions
      com_state_[0] = 0.0;
      com_state_[1] = 0.0;      

      // Estimate orientation       
      IMUOrientationEstimate();

    }else{
      // Update bias estimate
      x_bias_low_pass_filter->input(acc[0]);
      y_bias_low_pass_filter->input(acc[1]);
      z_bias_low_pass_filter->input(acc[2]);

      x_acc_bias = x_bias_low_pass_filter->output();
      y_acc_bias = y_bias_low_pass_filter->output(); 
      z_acc_bias = z_bias_low_pass_filter->output();           
    }

    // Get Local Acceleration Estimate
    x_acc_low_pass_filter->input(acc[0] - x_acc_bias);
    y_acc_low_pass_filter->input(acc[1] - y_acc_bias);
    z_acc_low_pass_filter->input(acc[2] - z_acc_bias);    


    // TEST
    //com_state_[4] = acc[0] - ini_acc_[0];
    com_state_[4] = x_acc_low_pass_filter->output();
    com_state_[5] = y_acc_low_pass_filter->output();

    com_state_[2] = com_state_[2] + com_state_[4]*mercury::servo_rate;
    com_state_[3] = com_state_[3] + com_state_[5]*mercury::servo_rate;

    com_state_[0] = com_state_[0] + com_state_[2]*mercury::servo_rate;
    com_state_[1] = com_state_[1] + com_state_[3]*mercury::servo_rate;

  // if(count % 100 == 0){
  //   dynacore::pretty_print(g_A, std::cout, "gravity_dir");
  //   printf("    gravity_mag = %0.4f \n", gravity_mag);
  //   printf("    theta_x = %0.4f \n", theta_x);    
  //   dynacore::pretty_print(g_A_local, std::cout, "rotated gravity_dir");
  // }    

    //count++;
}

void BasicAccumulation::IMUOrientationEstimate(){
  g_A.setZero();
  g_A[0] = -x_acc_bias;
  g_A[1] = -y_acc_bias;
  g_A[2] = -z_acc_bias;    
  gravity_mag = g_A.norm();
  g_A /= gravity_mag;

  dynacore::Quaternion q_world_Ry;
  dynacore::Quaternion q_world_roll;  

  // Prepare to rotate gravity vector
  g_A_local.w() = 0;
  g_A_local.x() = g_A[0];  g_A_local.y() = g_A[1]; g_A_local.z() = g_A[2];


  // Local xhat direction
  dynacore::Vect3 xhat_A; xhat_A.setZero(); xhat_A[0] = 1.0;
  // Compute Pitch to rotate
  theta_x = acos(xhat_A.dot(g_A));
  double pitch_val = (M_PI/2.0) - theta_x;
  //convert(0.0, pitch_val, 0.0, q_world_Ry);

  // Rotate gravity vector 
  //g_A_local = QuatMultiply( QuatMultiply(q_world_Ry, g_A_local), q_world_Ry.inverse());


}