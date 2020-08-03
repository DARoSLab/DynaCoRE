#include "Quadruped_Model.hpp"
#include "Quadruped_Dyn_Model.hpp"
#include "Quadruped_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Utils/utilities.hpp>
#include <Configuration.h>
#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Quadruped_Model::Quadruped_Model(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  if (!Addons::URDFReadFromFile (
              THIS_COM"/RobotSystems/Quadruped/quadruped.urdf", model_, true, false)) {
    std::cerr << "Error loading model ./quadruped.urdf" << std::endl;
    abort();
  }

  dyn_model_ = new Quadruped_Dyn_Model(model_);
  kin_model_ = new Quadruped_Kin_Model(model_);

  printf("[Quadruped Model] Contructed\n");
}

Quadruped_Model::~Quadruped_Model(){
  delete dyn_model_;
  delete kin_model_;
  delete model_;
}

void Quadruped_Model::UpdateSystem(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
    dynacore::Vector qddot = qdot; qddot.setZero();

  UpdateKinematicsCustom(*model_, &q, &qdot, &qddot);
  dyn_model_->UpdateDynamics(q, qdot);
  kin_model_->UpdateKinematics(q, qdot);
}

void Quadruped_Model::getCentroidInertia(dynacore::Matrix & Icent) const {
  kin_model_->getCentroidInertia(Icent);
}

void Quadruped_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const {
  Jcent.setZero();
  kin_model_->getCentroidJacobian(Jcent);
}

bool Quadruped_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const {
  return dyn_model_->getInverseMassInertia(Ainv);
}

bool Quadruped_Model::getMassInertia(dynacore::Matrix & A) const {
  return dyn_model_->getMassInertia(A);
}

bool Quadruped_Model::getGravity(dynacore::Vector & grav) const {
  return dyn_model_->getGravity(grav);
}

bool Quadruped_Model::getCoriolis(dynacore::Vector & coriolis) const {
  return dyn_model_->getCoriolis(coriolis);
}

void Quadruped_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
  //J = dynacore::Matrix::Zero(6, quadruped::num_qdot);
  kin_model_->getJacobian(link_id, J);
}

void Quadruped_Model::getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const{
    kin_model_->getJDotQdot(link_id, JDotQdot);
}

void Quadruped_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void Quadruped_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}
void Quadruped_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}
void Quadruped_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void Quadruped_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void Quadruped_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
  com_pos = kin_model_->com_pos_;
}

void Quadruped_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}
void Quadruped_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
  centroid_vel = kin_model_->centroid_vel_;
}
