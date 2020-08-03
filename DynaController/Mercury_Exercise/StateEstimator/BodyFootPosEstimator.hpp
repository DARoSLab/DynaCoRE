#ifndef BODY_FOOT_POSITON_ESTIMATOR
#define BODY_FOOT_POSITON_ESTIMATOR

#include <Utils/wrap_eigen.hpp>

class MoCapManager;
class RobotSystem;

class BodyFootPosEstimator{
public:
  BodyFootPosEstimator(RobotSystem*);
  ~BodyFootPosEstimator();

  void getMoCapBodyOri(dynacore::Quaternion & quat);

protected:
  MoCapManager* mocap_manager_;
};

#endif
