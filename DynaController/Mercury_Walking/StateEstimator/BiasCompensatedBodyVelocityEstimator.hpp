#ifndef BIAS_VELOCITY_ESTIMATION
#define BIAS_VELOCITY_ESTIMATION

#include <Utils/wrap_eigen.hpp>

#include <Filter/filters.hpp>

class BiasCompensatedBodyVelocityEstimator{
public:
  BiasCompensatedBodyVelocityEstimator();
  ~BiasCompensatedBodyVelocityEstimator();

  void EstimatorInitialization(const std::vector<double> & acc, const dynacore::Quaternion & q_in);
  void CoMStateInitialization(const dynacore::Vect3 & com_pos, 
                              const dynacore::Vect3 & com_vel);

  void setSensorData(const std::vector<double> & acc, const dynacore::Quaternion & q_in);

  void getEstimatedCoMState(dynacore::Vector & com_state);

protected:
    dynacore::Vector com_state_;
    dynacore::Vect3 ini_acc_;

    double bias_lp_frequency_cutoff;
    digital_lp_filter* x_bias_low_pass_filter;
    digital_lp_filter* y_bias_low_pass_filter;
    digital_lp_filter* z_bias_low_pass_filter;

    double x_acc_bias;
    double y_acc_bias;
    double z_acc_bias;

    digital_lp_filter* x_acc_low_pass_filter;
    digital_lp_filter* y_acc_low_pass_filter;
    digital_lp_filter* z_acc_low_pass_filter;        
    double lp_frequency_cutoff;

    dynacore::Vect3 acc_vec;
    dynacore::Quaternion Oq_B; // quaternion of the body frame w.r.t fixed frame
    dynacore::Matrix O_R_B; // Rotation Matrix

};

#endif
