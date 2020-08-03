#include "SagitP3_Model.hpp"
#include "SagitP3_Dyn_Model.hpp"
#include "SagitP3_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Configuration.h>
#include <Utils/utilities.hpp>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

SagitP3_Model::SagitP3_Model(){
    model_ = new Model();
    if (!Addons::URDFReadFromFile 
            (THIS_COM"RobotSystems/SagitP3/p3_model_orion.urdf", model_, true, 
             false)) {
        std::cerr << "Error loading model SagitP3.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new SagitP3_Dyn_Model(model_);
    kin_model_ = new SagitP3_Kin_Model(model_);

    printf("[SagitP3 Model] Contructed\n");
}

SagitP3_Model::~SagitP3_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void SagitP3_Model::UpdateSystem(const dynacore::Vector & q, 
        const dynacore::Vector & qdot){
    dynacore::Vector qddot = qdot; qddot.setZero();

    UpdateKinematics(*model_, q, qdot, qddot);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void SagitP3_Model::getCentroidInertia(dynacore::Matrix & Icent) const{
    kin_model_->getCentroidInertia(Icent);
}

void SagitP3_Model::getCentroidJacobian(dynacore::Matrix & Jcent) const{
    Jcent.setZero();
    kin_model_->getCentroidJacobian(Jcent);
}

bool SagitP3_Model::getInverseMassInertia(dynacore::Matrix & Ainv) const{
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool SagitP3_Model::getMassInertia(dynacore::Matrix & A) const {
    return dyn_model_->getMassInertia(A);
}

bool SagitP3_Model::getGravity(dynacore::Vector & grav) const {
    return dyn_model_->getGravity(grav);
}

bool SagitP3_Model::getCoriolis(dynacore::Vector & coriolis) const{
    return dyn_model_->getCoriolis(coriolis);
}

void SagitP3_Model::getFullJacobian(int link_id, dynacore::Matrix & J) const {
    kin_model_->getJacobian(link_id, J);
}

void SagitP3_Model::getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const {
    kin_model_->getJDotQdot(link_id, JDotQdot);
}

void SagitP3_Model::getPos(int link_id, dynacore::Vect3 & pos) const {
    kin_model_->getPos(link_id, pos);
}
void SagitP3_Model::getOri(int link_id, dynacore::Quaternion & ori) const {
    kin_model_->getOri(link_id, ori);
}

void SagitP3_Model::getLinearVel(int link_id, dynacore::Vect3 & vel) const {
    kin_model_->getLinearVel(link_id, vel);
}

void SagitP3_Model::getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {
    kin_model_->getAngularVel(link_id, ang_vel);
}

void SagitP3_Model::getCoMJacobian(dynacore::Matrix & J) const {
    J = dynacore::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(J);
}

void SagitP3_Model::getCoMPosition(dynacore::Vect3 & com_pos) const {
    com_pos = kin_model_->com_pos_;
}

void SagitP3_Model::getCoMVelocity(dynacore::Vect3 & com_vel) const {
    //kin_model_->getCoMVel(com_vel);
    com_vel = (kin_model_->centroid_vel_).tail(3);
}

void SagitP3_Model::getCentroidVelocity(dynacore::Vector & centroid_vel) const {
    centroid_vel = kin_model_->centroid_vel_;
}
