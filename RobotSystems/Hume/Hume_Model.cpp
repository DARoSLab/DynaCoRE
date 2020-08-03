#include "Hume_Model.h"

#include "Hume_Dyn_Model.h"
#include "Hume_Kin_Model.h"
#include "rbdl/urdfreader.h"
#include "utils/utilities.h"
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

HumeModel* HumeModel::GetHumeModel(){
    static HumeModel hume_model_;
    return & hume_model_;
}

HumeModel::HumeModel():WBOSC_Model(){
    model_ = new Model();
    
    if (!Addons::URDFReadFromFile (THIS_COM"test_config/hume.urdf", model_, false)) {
        std::cerr << "Error loading model ./hume.urdf" << std::endl;
        abort();
    }
    dyn_model_ = new Hume_Dyn_Model(model_);
    kin_model_ = new Hume_Kin_Model(model_);

    NUM_passive_qdot_ = 6;
    NUM_passive_q_ = 6;
    NUM_act_joints_ = 6;
    NUM_q_ = model_->q_size;
    NUM_qdot_ = model_->qdot_size;
    // dyn_model_->start();

    printf("[HumeModel] Contructed\n");
}

HumeModel::~HumeModel(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void HumeModel::getCentroidJacobian(sejong::Matrix & J){
    kin_model_->getCentroidJacobian(J);
}

void HumeModel::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
}

void HumeModel::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool HumeModel::InitializeDynamicsModel(const Vector & q, const Vector & qdot){
    // dyn_model_->UpdateDynamics();
    dyn_model_->UpdateDynamics(q, qdot);
}


bool HumeModel::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool HumeModel::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool HumeModel::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void HumeModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
    J = sejong::Matrix::Zero(6, model_->qdot_size);
    kin_model_->getJacobian(q, link_id, J);
}

void HumeModel::getPosition(const Vector & q,
                            int link_id, Vect3 & pos) {
    kin_model_->getPosition(q, link_id, pos);
}
void HumeModel::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
    kin_model_->getOrientation(q, link_id, ori);
}
void HumeModel::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
    kin_model_->getVelocity(q, qdot, link_id, vel);
}
void HumeModel::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void HumeModel::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void HumeModel::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
    kin_model_->getCoMPos(q, com_pos, update);
}

void HumeModel::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
