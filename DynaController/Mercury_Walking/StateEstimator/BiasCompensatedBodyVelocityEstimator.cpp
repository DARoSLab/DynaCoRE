#include "BiasCompensatedBodyVelocityEstimator.hpp"
#include <Utils/utilities.hpp>
#include <Configuration.h>
#include <Mercury/Mercury_Definition.h>

BiasCompensatedBodyVelocityEstimator::BiasCompensatedBodyVelocityEstimator(): com_state_(6), acc_vec(3){
  com_state_.setZero();
  acc_vec.setZero();

  // Initialize Rotation Paramters
  Oq_B.x() = 0.0;   Oq_B.y() = 0.0;   Oq_B.z() = 0.0;   Oq_B.w() = 1.0;
  O_R_B = dynacore::Matrix::Identity(3,3);

  // Bias Filter 
  bias_lp_frequency_cutoff = 2.0*3.1415*1.0; // 1Hz // (2*pi*frequency) rads/s 
  x_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  y_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
  z_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);  
  x_acc_bias = 0.0;
  y_acc_bias = 0.0;  
  z_acc_bias = 0.0;

  // Acceleration Low Pass Filter
  lp_frequency_cutoff = 2.0*3.1415*100; // 100Hz // (2*pi*frequency) rads/s
  x_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  y_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);
  z_acc_low_pass_filter = new digital_lp_filter(lp_frequency_cutoff, mercury::servo_rate);

}

BiasCompensatedBodyVelocityEstimator::~BiasCompensatedBodyVelocityEstimator(){}

void BiasCompensatedBodyVelocityEstimator::CoMStateInitialization(
        const dynacore::Vect3 & com_pos, 
        const dynacore::Vect3 & com_vel){

    com_state_[0] = com_pos[0];
    com_state_[1] = com_pos[1];
    com_state_[2] = com_vel[0];
    com_state_[3] = com_vel[1];
}

void BiasCompensatedBodyVelocityEstimator::getEstimatedCoMState(dynacore::Vector & com_state){
    com_state = com_state_;
}

void BiasCompensatedBodyVelocityEstimator::EstimatorInitialization(const std::vector<double> & acc, const dynacore::Quaternion & q_in){
  // Get acceleration vector
  for(size_t i = 0; i < acc.size(); i++){
    acc_vec[i] = -acc[i];
  }
  // Rotate to world frame.
  O_R_B = Oq_B.toRotationMatrix();
  acc_vec = O_R_B*acc_vec;

  // Update bias estimate
  x_bias_low_pass_filter->input(acc_vec[0]);
  y_bias_low_pass_filter->input(acc_vec[1]);
  z_bias_low_pass_filter->input(acc_vec[2]);

  x_acc_bias = x_bias_low_pass_filter->output();
  y_acc_bias = y_bias_low_pass_filter->output(); 
  z_acc_bias = z_bias_low_pass_filter->output();    

  // Reset local velocities
  com_state_[2] = 0.0;
  com_state_[3] = 0.0;   

  // Clear Acceleration Low Pass Filters
  x_acc_low_pass_filter->clear();
  y_acc_low_pass_filter->clear();
  z_acc_low_pass_filter->clear();   
}

void BiasCompensatedBodyVelocityEstimator::setSensorData(const std::vector<double> & acc, const dynacore::Quaternion & q_in){
  // Get acceleration vector
  for(size_t i = 0; i < acc.size(); i++){
    acc_vec[i] = -acc[i];
  }
  // Rotate to world frame.
  O_R_B = Oq_B.toRotationMatrix();
  acc_vec = O_R_B*acc_vec;

  // Update bias estimate
  x_bias_low_pass_filter->input(acc_vec[0]);
  y_bias_low_pass_filter->input(acc_vec[1]);
  z_bias_low_pass_filter->input(acc_vec[2]);

  x_acc_bias = x_bias_low_pass_filter->output();
  y_acc_bias = y_bias_low_pass_filter->output(); 
  z_acc_bias = z_bias_low_pass_filter->output();           

  // Get Local Acceleration Estimate
  x_acc_low_pass_filter->input(acc_vec[0] - x_acc_bias);
  y_acc_low_pass_filter->input(acc_vec[1] - y_acc_bias);
  z_acc_low_pass_filter->input(acc_vec[2] - z_acc_bias);    

  // Estimate CoM Body Accelerations
  com_state_[4] = x_acc_low_pass_filter->output();
  com_state_[5] = y_acc_low_pass_filter->output();

  com_state_[2] = com_state_[2] + com_state_[4]*mercury::servo_rate;
  com_state_[3] = com_state_[3] + com_state_[5]*mercury::servo_rate;

  com_state_[0] = com_state_[0] + com_state_[2]*mercury::servo_rate;
  com_state_[1] = com_state_[1] + com_state_[3]*mercury::servo_rate;
}