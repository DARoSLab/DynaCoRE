#ifndef EKF_LIP_ROTELLA_ESTIMATOR
#define EKF_LIP_ROTELLA_ESTIMATOR

#include "EKF_PoseEstimator.hpp"
#include <rbdl/rbdl.h>
#include <Filter/filters.hpp>


/* This estimator was based on:

Rotella, Nicholas, et al. 
"State estimation for a humanoid robot." 
Intelligent Robots and Systems (IROS 2014), 
2014 IEEE/RSJ International Conference on. IEEE, 2014.

and

Bloesch, Michael, et al. 
"State estimation for legged robots-consistent fusion of leg kinematics and IMU." 
Robotics 17 (2013): 17-24.

But we have since extensively modified it to include body velocity and LIPM dynamics

*/

class EKF_LIPRotellaEstimator :public EKF_PoseEstimator{
public:
  EKF_LIPRotellaEstimator();	
  ~EKF_LIPRotellaEstimator();
  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool & left_foot_contact,
                             const bool & right_foot_contact,
                             const dynacore::Vector & joint_values);

  virtual void setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             const dynacore::Vector & joint_values,
                             const dynacore::Vector & joint_velocity_values);

  virtual void EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel);


  virtual void EstimatorInitializationWithCOMHeight(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel,
                                       const double com_height);

  virtual void resetFilter();

  dynacore::Matrix getSkewSymmetricMatrix(dynacore::Vector & vec_in);


  void handleFootContacts();
  void computeNewFootLocations(const int foot_link_id);

  void setStateVariablesToPrior();
  void setStateVariablesToPredicted();
  
  void showPrintOutStatements();

  void showDebugStatements();  

  void getMatrix_L_c(const dynacore::Quaternion & q_in, dynacore::Matrix & L_c_mat);
  void getMatrix_Q(dynacore::Matrix & Q_mat);
  void statePredictionStep();
  void covariancePredictionStep();  

  void getCurrentBodyFrameFootPosition(const int foot_link_id, dynacore::Vector & foot_pos_B);

  void getBodyVelFromKinematics(dynacore::Vector & O_body_vel);

  void getCoMVelFromKinematics(dynacore::Vector & O_com_vel);


  void updateStatePosterior();

  void predictionStep();
  void updateStep();
  void doFilterCalculations(); 



  void calibrateOrientationFromGravity();

protected:
  dynacore::Vector O_p_l; // global left foot position
  dynacore::Vector O_p_r; // global right foot position   

  dynacore::Vector B_bf;  // imu frame acceleration bias
  dynacore::Vector B_bw;  // imu frame angular velocity bias  

  dynacore::Vector f_imu;  // imu frame acceleration
  dynacore::Vector omega_imu;  // imu frame angular velocity  

  dynacore::Vector f_imu_input;  // imu frame acceleration
  dynacore::Vector omega_imu_input;  // imu frame angular velocity  

  bool initial_foot_contact; // checks whether an initial foot contact at startup has occured for the first time.

  bool lf_contact; // value of left foot contact
  bool rf_contact; // value of right foot contact

  bool prev_lf_contact; // previous value of the left foot contact
  bool prev_rf_contact; // previous value of the right foot contact

  RigidBodyDynamics::Model* robot_model;

  dynacore::Vector Q_config; // configuration of the robot_model
  dynacore::Vector Q_config_dot; // configuration velocity of the robot model


  void _SetupParameters();
  int count;

  double lipm_height;

  // EKF Variables-----------------------------------
  double local_gravity;
  dynacore::Vector gravity_vec;
  double dt;

  int dim_states;
  int dim_rvq_states;
  int dim_process_errors;
  int dim_error_states;  
  int dim_obs;  
  int dim_inputs;

  dynacore::Matrix C_rot; // rotation matrix from inerital frame to body frame.

  dynacore::Matrix F_c; // contiouous prediction Jacobian. aka: f_x (linearized error state dynamics)
  dynacore::Matrix L_c; // continuous process error Jacobian
  dynacore::Matrix Q_c; // continuous process noise
  double wf_intensity;  // imu process noise intensity
  double ww_intensity;  // angular velocity noise intensity  
  double wp_l_intensity;  // left foot location noise intensity  
  double wp_r_intensity;  // right foot location noise intensity

  double wp_intensity_default; // default foot location noise intensity
  double wp_intensity_unknown; // noise intensity when there is no foot contact

  double wbf_intensity; // imu bias intensity
  double wbw_intensity; // angular velocity bias intensity

  dynacore::Matrix F_k; // discretized error state prediction matrix
  dynacore::Matrix H_k; // discretized error state observation matrix

  dynacore::Matrix R_c; // Measurement noise covariance matrix
  dynacore::Matrix R_k; // Discretized measurement noise covariance matrix  
  double n_p;           // Measurement noise intensity


  double n_v_default;   // default noise intensity of body velocity measurement
  double n_v_unknown;   // unknown noise intensity from body velocity measurement
  double n_v;           // Body velocity from kinematics measurement noise intensity

  double n_com_default;   // default noise intensity of com velocity measurement
  double n_com_unknown;   // unknown noise intensity from com velocity measurement
  double n_com;           // CoM velocity from kinematics measurement noise intensity

  dynacore::Vector body_vel_kinematics; // kinematics and angular velocity based body velocity measurement
  dynacore::Vector body_com_vel_kinematics; // kinematics and angular velocity based com velocity measurement
  dynacore::Vector body_imu_vel; // kinematics and angular velocity based com velocity measurement

  dynacore::Matrix S_k; // Innovation (residual) covariance
  dynacore::Matrix K_k; // Kalman gain  

  dynacore::Matrix P_prior; // Prior covariance matrix
  dynacore::Matrix P_predicted; // Predicted covariance matrix  
  dynacore::Matrix P_posterior; // Posterior covariance matrix

  dynacore::Vector x_prior; // Prior EKF States
  dynacore::Vector x_predicted; // Predicted EKF States  
  dynacore::Vector x_posterior; // Posterior EKF States    

  dynacore::Vector delta_x_prior; // Prior error states
  dynacore::Vector delta_x_posterior; // Posterier error states
  dynacore::Vector delta_y; // prediction and measurement differences

  dynacore::Vector z_lfoot_pos_B; // Measured body frame left foot position
  dynacore::Vector z_rfoot_pos_B; // Measured body frame right foot position

  dynacore::Vector p_l_B; // Body frame left foot position
  dynacore::Vector p_r_B; // Body frame right foot position

  dynacore::Vector y_vec; // prediction and measurement differences
  double weight_body_vel_obs;
  double weight_com_vel_obs;

  dynacore::Vect3 delta_phi;

  // Initialize Orientation Calibration Variables
  double theta_x;
  double theta_y;  
  double gravity_mag; 
  double roll_value_comp;
  double pitch_value_comp;  

  dynacore::Vect3 g_B;
  dynacore::Vect3 g_B_local_vec; // rotated gravity direction 
  dynacore::Quaternion Oq_B_init; // initial quaternion of the body frame w.r.t fixed frame
  dynacore::Matrix OR_B_init; // initial Rot matrix of body w.r.t fixed frame
  dynacore::Quaternion q_world_Rx; 
  dynacore::Quaternion q_world_Ry; 

  double bias_lp_frequency_cutoff;
  digital_lp_filter* x_bias_low_pass_filter;
  digital_lp_filter* y_bias_low_pass_filter;
  digital_lp_filter* z_bias_low_pass_filter;
  double x_acc_bias;
  double y_acc_bias;
  double z_acc_bias;


};


#endif
