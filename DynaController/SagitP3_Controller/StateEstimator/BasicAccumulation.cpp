#include "BasicAccumulation.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/DataManager.hpp>


BasicAccumulation::BasicAccumulation():OriEstimator(){
}


BasicAccumulation::~BasicAccumulation(){}
void BasicAccumulation::EstimatorInitialization(const dynacore::Quaternion & ini_quat,
        const std::vector<double> & acc,
        const std::vector<double> & ang_vel){
    
    for(int i(0); i<3; ++i){
        global_ang_vel_[i] = ang_vel[i];
    }
    global_ori_ = ini_quat;

}

void BasicAccumulation::setSensorData(const std::vector<double> & acc,
                                      const std::vector<double> & ang_vel){

  // Convert body omega into a delta quaternion ------------------------------
  dynacore::Vect3 body_omega; body_omega.setZero();
  for(size_t i = 0; i < 3; i++){
    body_omega[i] = ang_vel[i];
  }
  dynacore::Quaternion delta_quat_body;
  dynacore::convert(body_omega*mercury::servo_rate, delta_quat_body);

  dynacore::Matrix R_global_to_imu = global_ori_.normalized().toRotationMatrix();
  global_ang_vel_ = R_global_to_imu * body_omega;

  // Perform orientation update via integration
  global_ori_ = dynacore::QuatMultiply(global_ori_, delta_quat_body); 
}
