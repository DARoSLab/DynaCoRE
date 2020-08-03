#include "EKF_LIPRotellaEstimator.hpp"
#include <Configuration.h>
#include <Utils/utilities.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>
#include <Utils/DataManager.hpp>

#include <ParamHandler/ParamHandler.hpp>
#include <rbdl/urdfreader.h>

using namespace RigidBodyDynamics;

EKF_LIPRotellaEstimator::EKF_LIPRotellaEstimator():Q_config(mercury::num_q),
	Q_config_dot(mercury::num_qdot),
	O_p_l(3),
	O_p_r(3),
	B_bf(3),
	B_bw(3),
	initial_foot_contact(false),
	lf_contact(false),
	rf_contact(false),
	prev_lf_contact(false),
	prev_rf_contact(false),
	f_imu(3),
	omega_imu(3)	
{

	lipm_height = 0.89; // Will be set by the initialization process;

	// Load Robot Model
	robot_model = new Model();
	rbdl_check_api_version (RBDL_API_VERSION);
	if (!Addons::URDFReadFromFile (THIS_COM"/RobotSystems/Mercury/mercury.urdf", robot_model, false)) {
		std::cerr << "Error loading model ./mercury.urdf" << std::endl;
		abort();
	}
	// --------------------
	printf("[EKF Rotella Estimator] Successfully loaded URDF robot model.");

	// Initialize Global Positions
	O_r = dynacore::Vector::Zero(3);
	O_v = dynacore::Vector::Zero(3);

	Q_config.setZero();
	Q_config_dot.setZero();

	// Initialize Orientations
	O_q_B.w() = 1;
	O_q_B.x() = 0.0; 	O_q_B.y() = 0.0; 	O_q_B.z() = 0.0;	
	Q_config[mercury::num_qdot] = 1.0; // Set orientation to identity.	



	// Initialize remaining state vectors
	O_p_l.setZero();
	O_p_r.setZero();
	B_bf.setZero();
	B_bw.setZero();

	// Initialize inputs
	f_imu.setZero();
	omega_imu.setZero();

	count = 0;
	// Initialize EKF Variables
	local_gravity = 9.81; 
	gravity_vec	= dynacore::Vector::Zero(3);
	gravity_vec[2] = local_gravity;

	dt = mercury::servo_rate;
	dim_states = O_r.size() + O_v.size() + 4 + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_rvq_states = O_r.size() + O_v.size() + 4;


	dim_error_states = O_r.size() + O_v.size() + 3 + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_process_errors = f_imu.size() + omega_imu.size() + O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size();
	dim_obs = O_p_l.size() + O_p_r.size() + O_v.size() + O_v.size();
	dim_inputs = f_imu.size() + omega_imu.size();


	C_rot = dynacore::Matrix::Identity(3,3);
	F_c = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	L_c = dynacore::Matrix::Zero(dim_error_states, dim_process_errors);	
	Q_c = dynacore::Matrix::Zero(dim_process_errors,dim_process_errors);

	F_k = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	H_k = dynacore::Matrix::Zero(dim_obs, dim_error_states);

	R_c = dynacore::Matrix::Zero(dim_obs, dim_obs);
	R_k = dynacore::Matrix::Zero(dim_obs, dim_obs);

	S_k = dynacore::Matrix::Zero(dim_obs, dim_obs);
	K_k = dynacore::Matrix::Zero(dim_error_states,  dim_obs);	

	P_prior = dynacore::Matrix::Zero(dim_error_states, dim_error_states);
	P_predicted = dynacore::Matrix::Zero(dim_error_states, dim_error_states);	
	P_posterior = dynacore::Matrix::Zero(dim_error_states, dim_error_states);	

	x_prior = dynacore::Vector::Zero(dim_states);
	x_predicted = dynacore::Vector::Zero(dim_states);
	x_posterior = dynacore::Vector::Zero(dim_states);	
	x_prior[9] = 1.0; // Set Quaternion to identity.
	x_posterior[9] = 1.0; // Set Quaternion to identity.


	delta_x_prior = dynacore::Vector::Zero(dim_error_states);
	delta_x_posterior = dynacore::Vector::Zero(dim_error_states);	

	delta_phi.setZero();

	delta_y = dynacore::Vector::Zero(dim_obs);

	z_lfoot_pos_B = dynacore::Vector::Zero(O_p_l.size());
	z_rfoot_pos_B = dynacore::Vector::Zero(O_p_r.size());

	p_l_B = dynacore::Vector::Zero(O_p_l.size());
	p_r_B = dynacore::Vector::Zero(O_p_r.size());

	body_vel_kinematics = dynacore::Vector::Zero(O_v.size());
	body_com_vel_kinematics = dynacore::Vector::Zero(O_v.size());	
	body_imu_vel = dynacore::Vector::Zero(O_v.size());

	y_vec = dynacore::Vector::Zero(dim_obs);

	// Initialize Covariance parameters
	// Values are from reference paper. Need to be changed to known IMU parameters
	ww_intensity = 0.000523;  // rad/s/sqrt(Hz) // angular velocity process noise intensity

	wp_intensity_default = 0.001;//0.001;   // m/sqrt(Hz)	 // default foot location noise intensity
	wp_intensity_unknown = 1000.0;   // m/sqrt(Hz)	 // noise intensity when there is no foot contact
	// Simulation params
	wf_intensity = 0.001;//0.00078;   // m/(s^2)/sqrt(Hz) // imu process noise intensity
	wbf_intensity = 10.0;	  // m/(s^3)/sqrt(Hz)  // imu bias intensity

	// Real sensor params
	// wf_intensity = 0.01;//0.00078;   // m/(s^2)/sqrt(Hz) // imu process noise intensity
	// wbf_intensity = 10.0;	  // m/(s^3)/sqrt(Hz)  // imu bias intensity
	wbw_intensity = 0.000618; // rad/(s^2)/sqrt(Hz)	 // ang vel bias intensity

    n_p = 0.01; // foot measurement noise intensity.

    //n_v_default = 0.01;   // default noise intensity of body velocity measurement
    n_v_default = 10.0;   // default noise intensity of body velocity measurement    
    n_v_unknown = 1000;   // unknown noise intensity from body velocity measurement
    
    n_com_default = 0.01;   // default noise intensity of body velocity measurement
    n_com_unknown = 1000;   // unknown noise intensity from body velocity measurement

    _SetupParameters();
    wp_l_intensity = wp_intensity_unknown;   // m/sqrt(Hz)	 // left foot location noise intensity
    wp_r_intensity = wp_intensity_unknown;   // m/sqrt(Hz)   // right foot location noise intensity

    n_v = n_v_unknown;    // Body velocity from kinematics measurement noise intensity
    n_com = n_com_default;    // Body velocity from kinematics measurement noise intensity

// Initialize Orientation Calibration Filters
	// Bias Filter 
	bias_lp_frequency_cutoff = 2.0*3.1415*1.0; // 1Hz // (2*pi*frequency) rads/s 
	x_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
	y_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);
	z_bias_low_pass_filter = new digital_lp_filter(bias_lp_frequency_cutoff, mercury::servo_rate);  

	// Initialize body orientation to identity.
	dynacore::Vect3 rpy_init; rpy_init.setZero();
	dynacore::convert(rpy_init, Oq_B_init); 
	OR_B_init = Oq_B_init.toRotationMatrix();

	// Initialize other calibration parameters
	gravity_mag = 9.81; // m/s^2;
	theta_x = 0.0;
	theta_y = 0.0;

	x_acc_bias = 0.0;
	y_acc_bias = 0.0;
	z_acc_bias = 0.0;
	
	g_B.setZero();
	g_B_local_vec.setZero();

	// Register data for logging

	DataManager::GetDataManager()->RegisterData(&O_r, DYN_VEC, "ekf_o_r", 3);
	DataManager::GetDataManager()->RegisterData(&O_v, DYN_VEC, "ekf_o_v", 3);
	DataManager::GetDataManager()->RegisterData(&O_q_B, QUATERNION, "ekf_o_q_b", 4);
	DataManager::GetDataManager()->RegisterData(&f_imu, DYN_VEC, "ekf_f_imu", 3);
	DataManager::GetDataManager()->RegisterData(&omega_imu, DYN_VEC, "ekf_omega_imu", 3);
	DataManager::GetDataManager()->RegisterData(&B_bf, DYN_VEC, "ekf_B_bf", 3);
	DataManager::GetDataManager()->RegisterData(&B_bw, DYN_VEC, "ekf_B_bw", 3);
	DataManager::GetDataManager()->RegisterData(&y_vec, DYN_VEC, "ekf_measured_diff", 6);

	DataManager::GetDataManager()->RegisterData(&z_lfoot_pos_B, DYN_VEC, "ekf_z_l_foot_B", 3);
	DataManager::GetDataManager()->RegisterData(&z_rfoot_pos_B, DYN_VEC, "ekf_z_r_foot_B", 3);
	DataManager::GetDataManager()->RegisterData(&p_l_B, DYN_VEC, "ekf_p_left_B", 3);
	DataManager::GetDataManager()->RegisterData(&p_r_B, DYN_VEC, "ekf_p_right_B", 3);

	DataManager::GetDataManager()->RegisterData(&O_p_l, DYN_VEC, "ekf_p_left_O", 3);
	DataManager::GetDataManager()->RegisterData(&O_p_r, DYN_VEC, "ekf_p_right_O", 3);


	DataManager::GetDataManager()->RegisterData(&body_vel_kinematics, DYN_VEC, "ekf_body_vel_kin", 3);
	DataManager::GetDataManager()->RegisterData(&body_com_vel_kinematics, DYN_VEC, "ekf_com_vel_kin", 3);

	DataManager::GetDataManager()->RegisterData(&body_imu_vel, DYN_VEC, "ekf_imu_vel", 3);

}

void EKF_LIPRotellaEstimator::_SetupParameters(){
    ParamHandler handler(MercuryConfigPath"ESTIMATOR_ekf_lipm.yaml");
    handler.getValue("n_p", n_p);
    handler.getValue("ww_intensity", ww_intensity);
    handler.getValue("wp_intensity_default", wp_intensity_default);
    handler.getValue("wp_intensity_unknown", wp_intensity_unknown);
    // Simulation params
    handler.getValue("wf_intensity", wf_intensity);
    handler.getValue("wbf_intensity", wbf_intensity);

    // Real sensor params
    handler.getValue("wbw_intensity", wbw_intensity);
    handler.getValue("n_v_default", n_v_default);
    handler.getValue("n_v_unknown", n_v_unknown); 

    printf("2\n");
    handler.getValue("n_com_default", n_com_default);
    handler.getValue("n_com_unknown", n_com_unknown);
    printf("2\n");
}


EKF_LIPRotellaEstimator::~EKF_LIPRotellaEstimator(){
	delete robot_model;
}

void EKF_LIPRotellaEstimator::EstimatorInitializationWithCOMHeight(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel,
                                       const double com_height){

	// Initialize linear inverted pendulum height
	lipm_height = com_height;
	EstimatorInitialization(initial_global_orientation, initial_imu_acc, initial_imu_ang_vel);

}

void EKF_LIPRotellaEstimator::EstimatorInitialization(const dynacore::Quaternion & initial_global_orientation,
                                       const std::vector<double> & initial_imu_acc,
                                       const std::vector<double> & initial_imu_ang_vel){
	

	// Initialize IMU values
	for(size_t i = 0; i < 3; i++){
		f_imu[i] = -initial_imu_acc[i];
		omega_imu[i] = initial_imu_ang_vel[i];		
	}

	// Initialize Covariance matrix prior
	getMatrix_L_c(O_q_B, L_c);
	getMatrix_Q(Q_c);
	P_prior = L_c*Q_c*L_c.transpose();// L_c*Q_c;

	// Initialize Global Orientation
	//O_q_B = initial_global_orientation;

	// // Initialize global orientation
	calibrateOrientationFromGravity();

	x_prior[O_r.size()+O_v.size()] = O_q_B.x();
	x_prior[O_r.size()+O_v.size()+1] = O_q_B.y();
	x_prior[O_r.size()+O_v.size()+2] = O_q_B.z();		
	x_prior[O_r.size()+O_v.size()+3] = O_q_B.w();		

}

void EKF_LIPRotellaEstimator::calibrateOrientationFromGravity(){
	//printf("[EKF Rotella Estimator] Initializing Orientation from IMU acceleration data.  The robot should not be moving \n");

	// Update bias estimate
	x_bias_low_pass_filter->input(f_imu[0]);
	y_bias_low_pass_filter->input(f_imu[1]);
	z_bias_low_pass_filter->input(f_imu[2]);

	x_acc_bias = x_bias_low_pass_filter->output();
	y_acc_bias = y_bias_low_pass_filter->output(); 
	z_acc_bias = z_bias_low_pass_filter->output(); 	

	// Finds The orientation of the body with respect to the fixed frame O ((^OR_B ). 
	// This assumes that the IMU can only sense gravity as the acceleration.
	//
	// The algorithm attempts to solve ^OR_B * f_b = f_o, 
	// where f_b is the acceleration of gravity in the body frame.
	// f_o is the acceleration of gravity in the fixed frame
	// In addition to ^OR_B acting as a change of frame formula, 
	// note that ^OR_B will also equal to the orientation of the body w.r.t to the fixed frame.

	// We will rotate f_b using an extrinsic rotation with global R_x (roll) and R_y (pitch) rotations
	// Thus, we will perform ^OR_y ^OR_x f_b = f_o.

	// Finally, note that ^OR_b = ^OR_y ^OR_x.
	// The resulting orientation will have the xhat component of ^OR_b (ie: ^OR_b.col(0)) to be always planar 
	// with the inertial x-z plane. 
	//
	// It is best to visualize the extrinsic rotation on paper for any given extrinsic roll then pitch operations

	g_B.setZero();
	g_B[0] = x_acc_bias; 
	g_B[1] = y_acc_bias;
	g_B[2] = z_acc_bias; // We expect a negative number if gravity is pointing opposite of the IMU zhat direction

	gravity_mag = g_B.norm();
	g_B /= gravity_mag;

	// Prepare to rotate gravity vector
	g_B_local_vec[0] = g_B[0];   g_B_local_vec[1] = g_B[1];   g_B_local_vec[2] = g_B[2];

	//---------------------------------------------------------------------------
	// Use Rx to rotate the roll and align gravity vector  -
	// Compute Roll to rotate
	// theta_x = atan(g_B[1]/g_B[2]);
	// double roll_val = theta_x;      

	// The following method can handle any initial vector due to gravity
	theta_x = atan2(g_B_local_vec[2], g_B_local_vec[1]); // Returns angle \in [-pi, pi] between z and y projected vectors.
	double roll_val = (-M_PI/2.0 - theta_x);      // (-pi/2 - theta_x)
	roll_value_comp = roll_val;

	//dynacore::convert(0.0, 0.0, roll_val, q_world_Rx);
	// Create Roll Quaternion
	q_world_Rx.w() = cos(roll_val/2.);;
	q_world_Rx.x() = sin(roll_val/2.);
	q_world_Rx.y() = 0;
	q_world_Rx.z() = 0;

	//Rotate gravity vector to align the yhat directions
	dynacore::Matrix Rx = q_world_Rx.normalized().toRotationMatrix();
	g_B_local_vec = Rx*g_B_local_vec;

	// Use Ry to rotate pitch and align gravity vector  ---------------------------
	// Compute Pitch to rotate
	// theta_y = atan(g_B_local.x()/g_B_local.z());
	// double pitch_val = -theta_y;

	// The following method can handle any initial vector due to gravity
	theta_y = atan2(g_B_local_vec[2], g_B_local_vec[0]); // Returns angle \in [-pi, pi] between z and x projected vectors.
	double pitch_val = -((-M_PI/2.0) - theta_y);   // This is actually -(-pi/2 - theta_y)
	pitch_value_comp = pitch_val;

	//dynacore::convert(0.0, pitch_val, 0.0, q_world_Ry);
	// Create Pitch Quaternion
	q_world_Ry.w() = cos(pitch_val/2.);
	q_world_Ry.x() = 0;
	q_world_Ry.y() = sin(pitch_val/2.);
	q_world_Ry.z() = 0;  

	// Rotate gravity vector to align the xhat directions
	dynacore::Matrix Ry = q_world_Ry.normalized().toRotationMatrix();
	g_B_local_vec = Ry*g_B_local_vec;  

	// Obtain initial body orientation w.r.t fixed frame.
	//Oq_B = q_y * q_x * q_b
	Oq_B_init = dynacore::QuatMultiply(q_world_Ry, q_world_Rx);
	// Set rotation matrix
	OR_B_init = Oq_B_init.normalized().toRotationMatrix();

	// Initialize orientation
	O_q_B = Oq_B_init;

    // dynacore::pretty_print(g_B, std::cout, "gravity_dir");
    // printf("    gravity_mag = %0.4f \n", gravity_mag);
    // printf("    theta_x = %0.4f \n", theta_x);    
    // printf("    theta_y = %0.4f \n", theta_y);        
    // printf("    roll_value_comp = %0.4f \n", roll_value_comp);
    // printf("    pitch_value_comp = %0.4f \n", pitch_value_comp);
    // dynacore::pretty_print(g_B_local_vec, std::cout, "rotated gravity vec");    

    // dynacore::pretty_print(Oq_B_init, std::cout, "Initial body orientation w.r.t fixed frame: ");
    // dynacore::pretty_print(OR_B_init, std::cout, "OR_B_init: ");
    // dynacore::pretty_print(O_q_B, std::cout, "Body orientation w.r.t fixed frame: ");
    // printf("\n");

}


void EKF_LIPRotellaEstimator::computeNewFootLocations(const int foot_link_id){
    // Find Foot body id
    unsigned int bodyid;
    switch(foot_link_id){
        case mercury_link::leftFoot:
            bodyid = robot_model->GetBodyId("lfoot");
            break;
        case mercury_link::rightFoot:
            bodyid = robot_model->GetBodyId("rfoot");
            break;
        default:
            printf("[EKF_LIPRotellaEstimator] Not a valid foot link id\n");
            exit(0);
    }

    // Set parameters up
    dynacore::Vect3 Local_CoM = robot_model->mFixedBodies[
        bodyid - robot_model->fixed_body_discriminator].mCenterOfMass;

	Q_config.head(O_r.size()) = O_r;
	Q_config[O_r.size()] = O_q_B.x();
	Q_config[O_r.size()+1] = O_q_B.y();		
	Q_config[O_r.size()+2] = O_q_B.z();
	Q_config[mercury::num_qdot] = O_q_B.w();	

	// Update new foot locations
	if (foot_link_id == mercury_link::leftFoot){
	    O_p_l = CalcBodyToBaseCoordinates(*robot_model, Q_config, bodyid, Local_CoM, true);		
		x_prior.segment(dim_rvq_states, O_p_l.size()) = O_p_l;	    
	}
	if (foot_link_id == mercury_link::rightFoot){
	    O_p_r = CalcBodyToBaseCoordinates(*robot_model, Q_config, bodyid, Local_CoM, true);		
		x_prior.segment(dim_rvq_states + O_p_l.size(), O_p_r.size()) = O_p_r;
	}	


}

void EKF_LIPRotellaEstimator::resetFilter(){
	// Initialize Covariance matrix prior
	getMatrix_L_c(O_q_B, L_c);
	getMatrix_Q(Q_c);
	P_prior = L_c*Q_c*L_c.transpose();

	// // Initialize state vector except for the orientation
	x_prior.head(O_r.size() + O_v.size()) = dynacore::Vector::Zero(O_r.size() + O_v.size());
	x_prior.tail(O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size()) = dynacore::Vector::Zero(O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size());

	setStateVariablesToPrior();
	// x_prior = dynacore::Vector::Zero(dim_states);
	// x_prior[9] = 1.0;
	body_vel_kinematics = dynacore::Vector::Zero(3);
	body_com_vel_kinematics = dynacore::Vector::Zero(3);	
	body_imu_vel = dynacore::Vector::Zero(3);

	computeNewFootLocations(mercury_link::leftFoot); // Update Left foot location
	computeNewFootLocations(mercury_link::rightFoot); // Update Right foot location


}

void EKF_LIPRotellaEstimator::setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool left_foot_contact,
                             const bool right_foot_contact,
                             const dynacore::Vector & joint_values,
                             const dynacore::Vector & joint_velocity_values){


	Q_config_dot.segment(mercury::num_virtual, mercury::num_act_joint) = joint_velocity_values;
	setSensorData(acc, acc_inc, ang_vel, left_foot_contact, right_foot_contact, joint_values);

}

void EKF_LIPRotellaEstimator::setSensorData(const std::vector<double> & acc,
                             const std::vector<double> & acc_inc,
                             const std::vector<double> & ang_vel,
                             const bool & left_foot_contact,
                             const bool & right_foot_contact,
                             const dynacore::Vector & joint_values){


	for(size_t i = 0; i < 3; i++){
		f_imu[i] = -acc[i];
		omega_imu[i] = ang_vel[i];		
	}	
	//f_imu[i] = acc[i];
	//f_imu[2] = -f_imu[2];

	lf_contact = left_foot_contact;
	rf_contact = right_foot_contact;	
	Q_config.segment(mercury::num_virtual, mercury::num_act_joint) = joint_values;

	// Note: the first time contact is activated, the estimator is reset.
	if ( (!initial_foot_contact) && ((lf_contact) || (rf_contact)) ) {
		printf("\n");
		printf("[EKF_LIPRotellaEstimator] initial contact triggered. EKF filter will be reset. \n");
		printf("\n");
		initial_foot_contact = true;
		resetFilter();
	}


	// Handle changes in foot contact
	handleFootContacts();	

	// Perform filter calculations given sensor data
	doFilterCalculations();

	//Print Statements
	if (count % 100 == 0){
		showPrintOutStatements();
		count = 0;
	}
	count++;

	// Update foot contact booleans
	prev_lf_contact = lf_contact;
	prev_rf_contact = rf_contact;	

}

void EKF_LIPRotellaEstimator::handleFootContacts(){
	// if the foot location is lost, set wp to a high number
	if (lf_contact){
		wp_l_intensity = wp_intensity_default;	
	}else{
		wp_l_intensity = wp_intensity_unknown;
	}
	// if the foot location is lost, set wp to a high number
	if (rf_contact){
		wp_r_intensity = wp_intensity_default;
	}else{
		wp_r_intensity = wp_intensity_unknown;
	}	

	// Set n_v to a high number if neither foot contact is available
	if (lf_contact || rf_contact){
		n_v = n_v_default;
		n_com = n_com_default;
	}else{
		n_v = n_v_unknown;
		n_com = n_com_unknown;
	}

	// Handle new contact detections
	// Check if a new foot location will be used for estimation
	if ((prev_lf_contact == false) && (lf_contact == true)){
		// printf("\n New Left foot contact\n");
		computeNewFootLocations(mercury_link::leftFoot); // Update Left foot location
		// Update Prior?
		//P_prior.block(9,9,3,3) = wp_intensity_default*dynacore::Matrix::Identity(3,3);

	}
	if ((prev_rf_contact == false) && (rf_contact == true)){
		// printf("\n New Right foot contact\n");
		computeNewFootLocations(mercury_link::rightFoot); // Update right foot location
		// Update Prior?
		//P_prior.block(12,12,3,3) = wp_intensity_default*dynacore::Matrix::Identity(3,3);				
	}

	// Handle loss of contact
	// Update Covariance when a loss in contact is detected
	if ((prev_lf_contact == true) && (lf_contact == false)){
		//P_prior.block(9,9,3,3) = wp_intensity_unknown*dynacore::Matrix::Identity(3,3);
	}
	if ((prev_rf_contact == true) && (rf_contact == false)){
		//P_prior.block(12,12,3,3) = wp_intensity_unknown*dynacore::Matrix::Identity(3,3);				
	}
}

dynacore::Matrix EKF_LIPRotellaEstimator::getSkewSymmetricMatrix(dynacore::Vector & vec_in){
	dynacore::Matrix ssm = dynacore::Matrix::Zero(3,3);
	ssm(0,1) = -vec_in[2]; ssm(0,2) = vec_in[1];
	ssm(1,0) = vec_in[2];  ssm(1,2) = -vec_in[0];	
	ssm(2,0) = -vec_in[1]; ssm(2,1) = vec_in[0];
	return ssm;
}

void EKF_LIPRotellaEstimator::setStateVariablesToPrior(){
	// Get prior values:
	O_r = x_prior.head(O_r.size());
	O_v = x_prior.segment(O_r.size(), O_v.size());
	O_q_B.x() = x_prior[O_r.size() + O_v.size()];
	O_q_B.y() = x_prior[O_r.size() + O_v.size() + 1];	
	O_q_B.z() = x_prior[O_r.size() + O_v.size() + 2];		
	O_q_B.w() = x_prior[O_r.size() + O_v.size() + 3];

	C_rot = O_q_B.toRotationMatrix();

	O_p_l = x_prior.segment(dim_rvq_states, O_p_l.size());
	O_p_r = x_prior.segment(dim_rvq_states + O_p_l.size(), O_p_r.size());	
	B_bf = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size(), B_bf.size());
	B_bw = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size() + B_bw.size(), B_bw.size());		

	B_bf = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size(), B_bf.size());
	B_bw = x_prior.segment(dim_rvq_states + O_p_l.size() + O_p_r.size() + B_bw.size(), B_bw.size());		

}

void EKF_LIPRotellaEstimator::setStateVariablesToPredicted(){
	O_r = x_predicted.head(O_r.size());
	O_v = x_predicted.segment(O_r.size(), O_v.size());
	O_q_B.x() = x_predicted[O_r.size() + O_v.size()];
	O_q_B.y() = x_predicted[O_r.size() + O_v.size() + 1];	
	O_q_B.z() = x_predicted[O_r.size() + O_v.size() + 2];		
	O_q_B.w() = x_predicted[O_r.size() + O_v.size() + 3];

	C_rot = O_q_B.toRotationMatrix();

	O_p_l = x_predicted.segment(dim_rvq_states, O_p_l.size());
	O_p_r = x_predicted.segment(dim_rvq_states + O_p_l.size(), O_p_r.size());	
	B_bf = x_predicted.segment(dim_rvq_states + O_p_l.size() + O_p_r.size(), B_bf.size());
	B_bw = x_predicted.segment(dim_rvq_states + O_p_l.size() + O_p_r.size() + B_bf.size(), B_bw.size());	
}

void EKF_LIPRotellaEstimator::statePredictionStep(){
	// Prepare state variables
	setStateVariablesToPrior();

	// Prepare filter input:
	//f_imu_input = C_rot * f_imu;
	f_imu_input = f_imu - B_bf;
//	omega_imu_input = omega_imu - B_bw;
	omega_imu_input = omega_imu;


	// Prepare rotation values
	dynacore::Quaternion B_q_omega;
	dynacore::Vect3 vec3_omega_input; vec3_omega_input.setZero();
	for(size_t i = 0; i < 3; i++){ 
		vec3_omega_input[i] = omega_imu_input[i];
	}
	dynacore::convert(vec3_omega_input*dt, B_q_omega);

	// Perform Discrete State prediction step;
	// predict position and velocity

    double omega(local_gravity/lipm_height);

     if (lf_contact){
         // Use IP With the left foot swing
         x_predicted.head(2) = O_r.head(2) + O_v.head(2)*dt + 0.5*dt*dt*omega*(O_r.head(2) - O_p_l.head(2));
         x_predicted[2] = O_r[2] + O_v[2]*dt;

         x_predicted.segment(3,2) = O_v + dt*omega*(O_r.head(2) - O_p_l.head(2));
         x_predicted[5] = O_v[5];

     }else if(rf_contact){
         // Use IP With the right foot swing		
         x_predicted.head(2) = O_r.head(2) + O_v.head(2)*dt + 0.5*dt*dt*omega*(O_r.head(2) - O_p_r.head(2));
         x_predicted[2] = O_r[2] + O_v[2]*dt;

         x_predicted.segment(3,2) = O_v + dt*omega*(O_r.head(2) - O_p_r.head(2));
         x_predicted[5] = O_v[5];
     }else{
         x_predicted.segment(0, O_r.size()) = O_r + O_v*dt + 0.5*dt*dt*(C_rot.transpose()*f_imu_input + gravity_vec);
         x_predicted.segment(O_r.size(), O_v.size()) = O_v + dt*(C_rot.transpose()*f_imu_input + gravity_vec);	
     }

	//x_predicted.segment(0, O_r.size()) = O_r + O_v*dt + 0.5*dt*dt*(C_rot.transpose()*f_imu_input + gravity_vec);
	//x_predicted.segment(O_r.size(), O_v.size()) = O_v + dt*(C_rot.transpose()*f_imu_input + gravity_vec);	

	// predict orientation
	dynacore::Quaternion q_predicted = 	dynacore::QuatMultiply(O_q_B, B_q_omega);
//	dynacore::Quaternion q_predicted = 	dynacore::QuatMultiply(B_q_omega, O_q_B);	

	x_predicted[O_r.size()+O_v.size()] = q_predicted.x();
	x_predicted[O_r.size()+O_v.size()+1] = q_predicted.y();
	x_predicted[O_r.size()+O_v.size()+2] = q_predicted.z();		
	x_predicted[O_r.size()+O_v.size()+3] = q_predicted.w();			
	
	// predict foot position and biases
	x_predicted.segment(dim_rvq_states, dim_states - dim_rvq_states) = x_prior.segment(dim_rvq_states, dim_states - dim_rvq_states); 

	// Set variables to predicted
	setStateVariablesToPredicted();
}

void EKF_LIPRotellaEstimator::getMatrix_L_c(const dynacore::Quaternion & q_in, dynacore::Matrix & L_c_mat){
	dynacore::Matrix C_mat = q_in.toRotationMatrix();
	L_c_mat = dynacore::Matrix::Zero(dim_error_states, dim_process_errors);

	// L_c_mat.block(O_r.size(), 0, 3, 3) = -C_mat.transpose();
	// L_c_mat.block(O_r.size() + O_v.size(), f_imu.size(), 3, 3) = -dynacore::Matrix::Identity(3,3);
	// L_c_mat.block(O_r.size() + O_v.size() + 3, f_imu.size() + omega_imu.size(), 3,3) = C_mat.transpose();
	// L_c_mat.block(O_r.size() + O_v.size() + 3 + O_p_l.size(), f_imu.size() + omega_imu.size() + O_p_l.size(), 3, 3) = C_mat.transpose();	
	// L_c_mat.block(O_r.size() + O_v.size() + 3 + O_p_l.size() + O_p_r.size(), f_imu.size() + omega_imu.size() + O_p_l.size() + O_p_r.size(), 3, 3) = dynacore::Matrix::Identity(3,3);	
	// L_c_mat.block(O_r.size() + O_v.size() + 3 + O_p_l.size() + O_p_r.size() + B_bw.size(), 
	// 			  f_imu.size() + omega_imu.size() + O_p_l.size() + O_p_r.size() + B_bw.size(), 3, 3) = dynacore::Matrix::Identity(3,3);		

    double omega(local_gravity/lipm_height);
     dynacore::Matrix hg_mat = dynacore::Matrix::Identity(3,3);
     hg_mat(0,0) = omega;
     hg_mat(1,1) = omega;
     if(lf_contact){
         L_c_mat.block(3, 0, 3, 3) = hg_mat;		
         L_c_mat.block(3, 9, 3, 3) = hg_mat;		
     }else if(rf_contact){
         L_c_mat.block(3, 0, 3, 3) = hg_mat;		
         L_c_mat.block(3, 12, 3, 3) = hg_mat;		
     }else{
         L_c_mat.block(3, 0, 3, 3) = -C_mat.transpose();		
     }

	//L_c_mat.block(3, 0, 3, 3) = -C_mat.transpose();		
	L_c_mat.block(6, 3, 3, 3) = -dynacore::Matrix::Identity(3,3);
	L_c_mat.block(9, 6, 3,3) = C_mat.transpose();
	L_c_mat.block(12, 9, 3, 3) = C_mat.transpose();	
	L_c_mat.block(15, 12, 3, 3) = dynacore::Matrix::Identity(3,3);	
	L_c_mat.block(18, 15, 3, 3) = dynacore::Matrix::Identity(3,3);		



}

void EKF_LIPRotellaEstimator::getMatrix_Q(dynacore::Matrix & Q_mat){
	Q_mat = dynacore::Matrix::Zero(dim_process_errors, dim_process_errors);
	Q_mat.block(0, 0, 3, 3) = wf_intensity*dynacore::Matrix::Identity(3,3);
	Q_mat.block(3, 3, 3, 3) = ww_intensity*dynacore::Matrix::Identity(3,3);
	Q_mat.block(6, 6, 3, 3) = wp_l_intensity*dynacore::Matrix::Identity(3,3);
	Q_mat.block(9, 9, 3, 3) = wp_r_intensity*dynacore::Matrix::Identity(3,3);
	Q_mat.block(12, 12, 3, 3) = wbf_intensity*dynacore::Matrix::Identity(3,3);
	Q_mat.block(15, 15, 3, 3) = wbw_intensity*dynacore::Matrix::Identity(3,3);

}

void EKF_LIPRotellaEstimator::covariancePredictionStep(){
	// Prepare Covariance Prediction Step
	// Construct linearized error dynamics matrix, F_c
	F_c = dynacore::Matrix::Zero(dim_error_states, dim_error_states); 
	// F_c.block(0, O_r.size(), 3, 3) = dynacore::Matrix::Identity(3, 3);
	// F_c.block(O_r.size(), O_r.size() + O_v.size(), 3, 3) = -C_rot.transpose()*getSkewSymmetricMatrix(f_imu_input);
	// F_c.block(O_r.size(), dim_rvq_states + O_p_l.size() + O_p_r.size(), 3, 3) = -C_rot.transpose();
	// F_c.block(O_r.size() + O_v.size(), O_r.size() + O_v.size(), 3, 3) = -getSkewSymmetricMatrix(omega_imu_input);
	// F_c.block(O_r.size() + O_v.size(), dim_error_states - B_bw.size(), B_bw.size(), B_bw.size()) = -dynacore::Matrix::Identity(3,3);

	F_c.block(0, 3, 3, 3) = dynacore::Matrix::Identity(3, 3);

     if(lf_contact){
         F_c.block(3, 0, 2, 2) = (lipm_height/local_gravity)*dynacore::Matrix::Identity(2, 2);		
         F_c.block(3, 9, 2, 2) = (lipm_height/local_gravity)*dynacore::Matrix::Identity(2, 2);
     }else if(rf_contact){
         F_c.block(3, 0, 3, 3) = (lipm_height/local_gravity)*dynacore::Matrix::Identity(2, 2);		
         F_c.block(3, 12, 3, 3) = (lipm_height/local_gravity)*dynacore::Matrix::Identity(2, 2);	
     }else{
         F_c.block(3, 6, 3, 3) = -C_rot.transpose()*getSkewSymmetricMatrix(f_imu_input);
         F_c.block(3, 15, 3, 3) = -C_rot.transpose();
     }

	//F_c.block(3, 6, 3, 3) = -C_rot.transpose()*getSkewSymmetricMatrix(f_imu_input);
	//F_c.block(3, 15, 3, 3) = -C_rot.transpose();
	F_c.block(6, 6, 3, 3) = -getSkewSymmetricMatrix(omega_imu_input);
	F_c.block(6, 18, 3,3) = -dynacore::Matrix::Identity(3,3);
	// F_c.block(6, 18, 3,3) = O_q_B.toRotationMatrix(); //-dynacore::Matrix::Identity(3,3);

	// Discretized linear error dynamics
	F_k = dynacore::Matrix::Identity(F_c.rows(), F_c.cols()) + F_c*dt;

	// Construct Process noise Jacobian Matrix, L_c
	getMatrix_L_c(O_q_B, L_c);
	getMatrix_Q(Q_c);

	// Perform Covariance prediction step
	P_predicted = F_k*P_prior*F_k.transpose() + F_k*L_c*Q_c*L_c.transpose()*F_k.transpose()*dt;
	//P_predicted = L_c*Q_c*L_c.transpose()*dt*dt;
}

void EKF_LIPRotellaEstimator::predictionStep(){
	statePredictionStep();
	covariancePredictionStep();
}

void EKF_LIPRotellaEstimator::getCurrentBodyFrameFootPosition(const int foot_link_id, dynacore::Vector & foot_pos_B){
    // Find Foot body id
    unsigned int bodyid;
    switch(foot_link_id){
        case mercury_link::leftFoot:
            bodyid = robot_model->GetBodyId("lfoot");
            break;
        case mercury_link::rightFoot:
            bodyid = robot_model->GetBodyId("rfoot");
            break;
        default:
            printf("[EKF_LIPRotellaEstimator] Not a valid foot link id\n");
            exit(0);
    }

    // Set parameters up
    dynacore::Vect3 Local_CoM = robot_model->mFixedBodies[
        bodyid - robot_model->fixed_body_discriminator].mCenterOfMass;

    // Align body frame to the fixed frame
	dynacore::Vector config_copy = Q_config;
	config_copy.head(O_r.size()) = dynacore::Vector::Zero(3);
	config_copy[O_r.size()] = 0.0;
	config_copy[O_r.size()+1] = 0.0;		
	config_copy[O_r.size()+2] = 0.0;
	config_copy[mercury::num_qdot] = 1.0;	

    foot_pos_B = CalcBodyToBaseCoordinates(*robot_model, config_copy, bodyid, Local_CoM, true);		
}

void EKF_LIPRotellaEstimator::getBodyVelFromKinematics(dynacore::Vector & O_body_vel){
	dynacore::Vect3 vel; vel.setZero();
    dynacore::Vect3 zero; zero.setZero();
    O_body_vel = dynacore::Vector::Zero(3);

    unsigned int bodyid;
    if (lf_contact && rf_contact){
    	bodyid = robot_model->GetBodyId("lfoot");
    }else if (lf_contact){
    	bodyid = robot_model->GetBodyId("lfoot");
    }else if (rf_contact){
    	bodyid = robot_model->GetBodyId("rfoot");
    }else{
       	bodyid = robot_model->GetBodyId("lfoot");
    }

    if(bodyid >=robot_model->fixed_body_discriminator){
        zero = robot_model->mFixedBodies[bodyid - robot_model->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  robot_model->mBodies[bodyid].mCenterOfMass;
    }

	dynacore::Vector config_copy = Q_config;

	config_copy.head(O_r.size()) = O_r;
	config_copy[O_r.size()] = O_q_B.x();
	config_copy[O_r.size()+1] = O_q_B.y();		
	config_copy[O_r.size()+2] = O_q_B.z();
	config_copy[mercury::num_qdot] = O_q_B.w();	

	dynacore::Vector config_dot_copy = Q_config_dot;
    for(int i(0); i<3; ++i){
        config_dot_copy[i] = 0.0; // set 0 velocities for the linear components
        config_dot_copy[i+3] = omega_imu_input[i];
    }


    vel = CalcPointVelocity (*robot_model, config_copy, config_dot_copy, bodyid, zero, true);

    // Set body velocity to negative of foot velocity
    for(size_t i = 0; i < 3; i++){
    	O_body_vel[i] = -vel[i];
    }

}

void EKF_LIPRotellaEstimator::getCoMVelFromKinematics(dynacore::Vector & O_com_vel){
    int start_idx = robot_model->GetBodyId("body");
    double mass = 0.0;
    double tot_mass = 0.0;
    dynacore::Vect3 link_vel; link_vel.setZero();
    O_com_vel = dynacore::Vector::Zero(3);


    // Copy configuration and velocity
    // Assumes that the following values have been updated
	dynacore::Vector config_copy = Q_config;
	config_copy.head(O_r.size()) = O_r;
	config_copy[O_r.size()] = O_q_B.x();
	config_copy[O_r.size()+1] = O_q_B.y();		
	config_copy[O_r.size()+2] = O_q_B.z();
	config_copy[mercury::num_qdot] = O_q_B.w();	

	dynacore::Vector config_dot_copy = Q_config_dot;
	config_dot_copy.head(3) = body_vel_kinematics.head(3);

    for(int i(0); i<3; ++i){
        config_dot_copy[i+3] = omega_imu_input[i];
    }



    link_vel = CalcPointVelocity ( *robot_model, config_copy, config_dot_copy, start_idx, robot_model->mBodies[start_idx].mCenterOfMass, true);
    // Find velocities of all the links and weight it by their mass.
    for (int i(start_idx); i< robot_model->mBodies.size() ; ++i){
        mass = robot_model->mBodies[i].mMass;
        // CoM velocity Update
        link_vel = CalcPointVelocity ( *robot_model, config_copy, config_dot_copy, i, robot_model->mBodies[i].mCenterOfMass, false);
        O_com_vel += mass * link_vel;
        tot_mass += mass;
    }
    O_com_vel /= tot_mass;
}

void EKF_LIPRotellaEstimator::updateStep(){
	// Construct Noise Matrix R
	R_c.block(0,0,6,6) = n_p*dynacore::Matrix::Identity(6,6);

	// Get body velocity estimate from kinematics
	getBodyVelFromKinematics(body_vel_kinematics);
	R_c.block(6,6,3,3) = n_v*dynacore::Matrix::Identity(3,3);

	// Get CoM velocity estimate from kinematics
	getCoMVelFromKinematics(body_com_vel_kinematics);
	R_c.block(9,9,3,3) = n_com*dynacore::Matrix::Identity(3,3);

	// Get velocity estimate from the IMU
	body_imu_vel += (C_rot.transpose()*f_imu + gravity_vec)*dt;

	R_k = R_c/dt;

	// Construct Discretized Observation Matrix
	H_k.block(0,0,3,3) = -C_rot;
	p_l_B = C_rot*(O_p_l - O_r); // the left foot position in body frame
	H_k.block(0,6,3,3) = getSkewSymmetricMatrix(p_l_B);	
	H_k.block(0,9,3,3) = C_rot;

	H_k.block(3,0,3,3) = -C_rot;
	p_r_B = C_rot*(O_p_r - O_r); // the right foot position in body frame	
	H_k.block(3,6,3,3) = getSkewSymmetricMatrix(p_r_B);	
	H_k.block(3,12,3,3) = C_rot;

	H_k.block(6, 3, 3, 3) = dynacore::Matrix::Identity(3,3); 	// Body velocity estimate
	H_k.block(9, 3, 3, 3) = dynacore::Matrix::Identity(3,3);	// CoM velocity estimate

	//Compute the Kalman Gain
	S_k = H_k*P_predicted*H_k.transpose() + R_k;
	K_k = P_predicted*H_k.transpose()*(S_k.inverse());

	// Perform Measurement Using Kinematics
	getCurrentBodyFrameFootPosition(mercury_link::leftFoot, z_lfoot_pos_B);
	getCurrentBodyFrameFootPosition(mercury_link::rightFoot, z_rfoot_pos_B);

	// Measurement error vector
	// y_vec = (z - h())
	y_vec.head(3) = z_lfoot_pos_B - p_l_B;
	y_vec.segment(3, 3) = z_rfoot_pos_B - p_r_B;
	y_vec.segment(6,3) = body_vel_kinematics - O_v;
	y_vec.segment(9,3) = body_com_vel_kinematics - O_v;


	delta_x_posterior = K_k*y_vec;
	//delta_y = H_k*delta_x_posterior;

	// Update State
	updateStatePosterior();

	// Update Covariance Matrix
	P_posterior = (dynacore::Matrix::Identity(dim_error_states, dim_error_states) - K_k*H_k)*P_predicted;
	P_prior = P_posterior;


	// Update Prior
	//x_prior = x_predicted;
	//P_prior = P_predicted;
}

void EKF_LIPRotellaEstimator::updateStatePosterior(){
	for(size_t i = 0; i < delta_x_posterior.size(); i++){
		if (std::isnan(delta_x_posterior[i])){
			printf("Delta x change Contains isnan value. Will replace this with 0.0 \n");
			delta_x_posterior[i] = 0.0;
			printf("Here are the debug statements \n");
			showDebugStatements();
			printf("Quitting program... \n");
			exit(0);
		}
	}

	x_posterior = x_predicted;
	x_posterior.head(O_r.size() + O_v.size()) += delta_x_posterior.head(O_r.size() + O_v.size());
	x_posterior.tail(O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size()) += delta_x_posterior.tail(O_p_l.size() + O_p_r.size() + B_bf.size() + B_bw.size());

	// Prepare quaternion update
	dynacore::Quaternion q_prediction, q_change, q_posterior;	
	q_prediction.x() = x_predicted[O_r.size()+O_v.size()];
	q_prediction.y() = x_predicted[O_r.size()+O_v.size()+1];
	q_prediction.z() = x_predicted[O_r.size()+O_v.size()+2];		
	q_prediction.w() = x_predicted[O_r.size()+O_v.size()+3];

	// Convert delta phi change to quaternion
	delta_phi[0] = delta_x_posterior[O_r.size() + O_v.size()];
	delta_phi[1] = delta_x_posterior[O_r.size() + O_v.size() + 1];	
	delta_phi[2] = delta_x_posterior[O_r.size() + O_v.size() + 2];	
	dynacore::convert(delta_phi, q_change);

	// Perform Quaternion update
	q_posterior = dynacore::QuatMultiply(q_change, q_prediction);
	q_posterior = q_prediction;

	x_posterior[O_r.size()+O_v.size()] = q_posterior.x();
	x_posterior[O_r.size()+O_v.size()+1] = q_posterior.y();
	x_posterior[O_r.size()+O_v.size()+2] = q_posterior.z();		
	x_posterior[O_r.size()+O_v.size()+3] = q_posterior.w();				

	x_prior = x_posterior;
}



void EKF_LIPRotellaEstimator::doFilterCalculations(){
	predictionStep();
	updateStep();
}



void EKF_LIPRotellaEstimator::showPrintOutStatements(){
	//printf("\n");
	// printf("[EKF Rotella Estimator]\n");
	// dynacore::pretty_print(f_imu, std::cout, "body frame f_imu");
	// dynacore::pretty_print(body_imu_vel, std::cout, "body_imu_vel");

	// dynacore::pretty_print(omega_imu, std::cout, "body frame omega_imu");
	// dynacore::Vector a_o = (C_rot.transpose()*f_imu_input + gravity_vec);
	// dynacore::pretty_print(O_q_B, std::cout, "O_q_B");	
	// dynacore::pretty_print(C_rot, std::cout, "C_rot");	
	// dynacore::pretty_print(a_o, std::cout, "inertial frame f_imu");

	// dynacore::pretty_print(omega_imu, std::cout, "body frame angular velocity");			

	// dynacore::pretty_print(f_imu_input, std::cout, "f_imu_input");
	// dynacore::pretty_print(omega_imu_input, std::cout, "omega_imu_input");

	// dynacore::pretty_print(x_prior, std::cout, "x_prior");
	// dynacore::pretty_print(x_predicted, std::cout, "x_predicted");
	// dynacore::pretty_print(y_vec, std::cout, "y_vec");
	// dynacore::pretty_print(delta_x_posterior, std::cout, "delta_x_posterior");

	//dynacore::pretty_print(R_k, std::cout, "R_k");	
	// dynacore::pretty_print(delta_phi, std::cout, "delta_phi");	
	// dynacore::pretty_print(x_posterior, std::cout, "x_posterior");


	// dynacore::pretty_print(F_c, std::cout, "F_c");
	//dynacore::pretty_print(F_k, std::cout, "F_k");	
	// dynacore::pretty_print(L_c, std::cout, "L_c");		
	// dynacore::pretty_print(Q_c, std::cout, "Q_c");		
	// dynacore::pretty_print(P_prior, std::cout, "P_prior");		
	// dynacore::pretty_print(H_k, std::cout, "H_k");		

	// dynacore::pretty_print(K_k, std::cout, "K_k");

	// dynacore::pretty_print(z_lfoot_pos_B, std::cout, "z_lfoot");
	// dynacore::pretty_print(z_rfoot_pos_B, std::cout, "z_rfoot");
	//dynacore::pretty_print(y_vec, std::cout, "y_vec");	


	// printf("Left Foot contact = %d \n", lf_contact);
	// printf("Right Foot contact = %d \n", rf_contact);	
	// dynacore::pretty_print(Q_config, std::cout, "Q_config");			
}

void EKF_LIPRotellaEstimator::showDebugStatements(){
	dynacore::pretty_print(f_imu, std::cout, "f_imu");				
	dynacore::pretty_print(omega_imu, std::cout, "omega_imu");			

	dynacore::pretty_print(Q_config, std::cout, "Q_config");	
	dynacore::pretty_print(Q_config_dot, std::cout, "Q_config_dot");		

	dynacore::pretty_print(x_prior, std::cout, "x_prior");	
	dynacore::pretty_print(x_predicted, std::cout, "x_predicted");	
	dynacore::pretty_print(x_posterior, std::cout, "x_posterior");	

	dynacore::pretty_print(S_k, std::cout, "S_k");		
	dynacore::pretty_print(K_k, std::cout, "K_k");	
}
