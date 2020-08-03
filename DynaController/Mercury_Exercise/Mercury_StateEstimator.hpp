#ifndef STATE_ESTIMATOR_MERCURY
#define STATE_ESTIMATOR_MERCURY

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>

class Mercury_StateProvider;
class RobotSystem;
class filter;
class OriEstimator;
class BodyFootPosEstimator;
class Mercury_SensorData;

class Mercury_StateEstimator{
public:
  Mercury_StateEstimator(RobotSystem* robot);
  ~Mercury_StateEstimator();

  void Initialization(Mercury_SensorData* );
  void Update(Mercury_SensorData* );
  void setFloatingBase(int base_cond){ base_cond_ = base_cond; }

protected:
  int base_cond_;
  double initial_height_;
  int fixed_foot_;
  dynacore::Vect3 foot_pos_;
  Mercury_StateProvider* sp_;
  RobotSystem* robot_sys_;

  OriEstimator* ori_est_;
  BodyFootPosEstimator* body_foot_est_;
  std::vector<filter*> jvel_filter_;
};

#endif
