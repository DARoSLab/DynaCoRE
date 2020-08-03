#ifndef EKF_POSE_ESTIMATOR
#define EKF_POSE_ESTIMATOR

#include <Utils/wrap_eigen.hpp>

class EKF_PoseEstimator{
public:
  EKF_PoseEstimator(){}
  virtual ~EKF_PoseEstimator(){}

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool & left_foot_contact,
                             const bool & right_foot_contact,
                             const dynacore::Vector & joint_values) = 0;

 virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             const dynacore::Vector & joint_values,
                             const dynacore::Vector & joint_velocity_values) = 0;

  virtual void EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel) = 0;

  virtual void EstimatorInitializationWithCOMHeight(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel,
                                       const double com_height) {}


  virtual void resetFilter() = 0;

  void getEstimatedState(dynacore::Vect3 & global_position,    
                         dynacore::Vect3 & global_velocity,
                         dynacore::Quaternion & global_orientation){
    for (size_t i = 0; i < 3; i++){
      global_position[i] = O_r[i];
      global_velocity[i] = O_v[i];      
    }
    global_orientation = O_q_B;    

  }

protected:
  dynacore::Vector O_r; // global body position
  dynacore::Vector O_v; // global body velocity
  dynacore::Quaternion O_q_B; // global body orientation

};

#endif
