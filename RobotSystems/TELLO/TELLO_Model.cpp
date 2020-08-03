#include "TELLO_Model.hpp"
#include "TELLO_Dyn_Model.hpp"
#include "TELLO_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>
#include <Utils/utilities.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

TELLO_Model::TELLO_Model(){
    model_ = new Model();
    if (!Addons::URDFReadFromFile 
            //(THIS_COM"RobotSystems/TELLO/TELLO_URDF.urdf", model_, true, false)) {
            (THIS_COM"RobotSystems/TELLO/TELLO_humanoid.urdf", model_, true, true)) {
        std::cerr << "Error loading model TELLO_URDF.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new TELLO_Dyn_Model(model_);
    kin_model_ = new TELLO_Kin_Model(model_);

    printf("[TELLO Model] Contructed\n");
}

TELLO_Model::~TELLO_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void TELLO_Model::UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void TELLO_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void TELLO_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool TELLO_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool TELLO_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool TELLO_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool TELLO_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void TELLO_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    kin_model_->getJacobian(link_id, J);
}

void TELLO_Model::getFullJDotQdot(int link_id, dynacore::Vector & Jdotqdot) const {
    kin_model_->getJDotQdot(link_id, Jdotqdot);
}

void TELLO_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void TELLO_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void TELLO_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void TELLO_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void TELLO_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void TELLO_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void TELLO_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    kin_model_->getCoMVel(com_vel);
}

void TELLO_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
