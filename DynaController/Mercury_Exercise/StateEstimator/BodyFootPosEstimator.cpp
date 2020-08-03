#include "BodyFootPosEstimator.hpp"
#include <Mercury_Controller/MoCapManager.hpp>

BodyFootPosEstimator::BodyFootPosEstimator(RobotSystem* robot){
  mocap_manager_ = new MoCapManager(robot);
  mocap_manager_->start();

}
BodyFootPosEstimator::~BodyFootPosEstimator(){
  delete mocap_manager_;
}


void BodyFootPosEstimator::getMoCapBodyOri(dynacore::Quaternion & quat){
  quat = mocap_manager_->body_quat_;
}

