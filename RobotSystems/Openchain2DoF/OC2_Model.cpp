#include "OC2_Model.hpp"
#include "OC2_Dyn_Model.hpp"
#include "OC2_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Utils/utilities.hpp>

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

OC2Model* OC2Model::GetOC2Model(){
    static OC2Model draco_model_;
    return & draco_model_;
}

OC2Model::OC2Model(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);
  model_->gravity = Vector3d (0.,0.,  -9.81);

  Matrix3d inertia;
  Vector3d com_pos, link_length;
  com_pos.setZero(); link_length.setZero();
  Joint joint_ry = Joint (JointTypeRevolute, Vector3d (0., 1., 0.) );
  Body link1, link2, link_ee;

  //////////////////////////////////////////////////////
  ///                   Parameters                   ///
  //////////////////////////////////////////////////////
  double mass (1.7);
  com_pos[0] = 0.15;
  link_length[0] = 0.3;
  // xx, yy, zz, xy, yz, zx
  inertia <<
    0.001, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.2;

  //////////////////////////////////////////////////////
  ///                 Assemble Model                 ///
  //////////////////////////////////////////////////////

  // link 1
  link1 = Body (mass, com_pos, inertia);
  int link1_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_ry, link1, "link1");
  printf("link 1 id: %i \n", link1_id);

  // link 2
  link2 = Body (mass, com_pos, inertia);
  int link2_id = model_->AddBody(link1_id, Xtrans(link_length), joint_ry, link1, "link2");
  printf("link 2 id: %i \n", link2_id);

  // Fixed Joint (EE)
  Joint fixed_joint = Joint(JointTypeFixed);
  com_pos.setZero();
  Vector3d gyration_radii; gyration_radii.setZero();
  double mass_zero(0.000000001);
  link_ee = Body(mass_zero, com_pos, gyration_radii);
  int ee_id = model_->AddBody(link2_id, Xtrans(link_length), fixed_joint, link_ee, "link_ee");
  printf("link ee id: %i \n", ee_id);

  //////////////////////////////////////////////////////
  ///            End of Assemble Model               ///
  //////////////////////////////////////////////////////

    dyn_model_ = new OC2_Dyn_Model(model_);
    kin_model_ = new OC2_Kin_Model(model_);

    printf("[OC2 Model] Contructed\n");
}

OC2Model::~OC2Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void OC2Model::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void OC2Model::getCentroidInertia(sejong::Matrix & Icent){
  sejong::Matrix Icm_tmp;
  kin_model_->getCentroidInertia(Icm_tmp);
  // sejong::pretty_print(Icm_tmp, std::cout, "Icm");
  Icent = Icm_tmp.block(3, 3, 3, 3);
  Icent(2,2) = Icm_tmp(2,2);
}

void OC2Model::getCentroidJacobian(sejong::Matrix & Jcent){
  sejong::Matrix Jcent_tmp(6, NUM_QDOT);
  Jcent_tmp.setZero();
  kin_model_->getCentroidJacobian(Jcent_tmp);

  Jcent = Jcent_tmp.block(3, 0, 3, NUM_QDOT);
  Jcent.block(1,0, 1, NUM_QDOT) = Jcent_tmp.block(5, 0, 1, NUM_QDOT); //Z
  Jcent.block(2,0, 1, NUM_QDOT) = Jcent_tmp.block(2, 0, 1, NUM_QDOT); //Ry
}

void OC2Model::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool OC2Model::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool OC2Model::getMassInertia(sejong::Matrix & A) {
    return dyn_model_->getMassInertia(A);
}

bool OC2Model::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool OC2Model::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void OC2Model::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  sejong::Matrix Jtmp(6, NUM_QDOT);
  Jtmp.setZero();
  kin_model_->getJacobian(q, link_id, Jtmp);

  // X, Z, Ry
  J = Jtmp.block(3,0, 3, NUM_QDOT);
  J.block(1,0, 1, NUM_QDOT) = Jtmp.block(5, 0, 1, NUM_QDOT); // Z
  J.block(2,0, 1, NUM_QDOT) = Jtmp.block(1, 0, 1, NUM_QDOT); // Ry
}
void OC2Model::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  sejong::Matrix Jdot_analytic;
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot_analytic);

  // X, Z, Ry
  Jdot = Jdot_analytic.block(3,0, 3, NUM_QDOT); // X, Y, Z
  Jdot.block(1,0, 1, NUM_QDOT) = Jdot_analytic.block(5, 0, 1, NUM_QDOT); // Z
  Jdot.block(2,0, 1, NUM_QDOT) = Jdot_analytic.block(1, 0, 1, NUM_QDOT); // Ry
}

void OC2Model::getPosition(const Vector & q,
                             int link_id, Vect3 & pos) {
  // X, Z, Ry
  sejong::Vect3 pos_tmp;
  kin_model_->getPosition(q, link_id, pos_tmp);
  sejong::Quaternion ori;
  kin_model_->getOrientation(q, link_id, ori);

  pos[0] = pos_tmp[0];
  pos[1] = pos_tmp[2];
  pos[2] = 2. * asin(ori.y());
}
void OC2Model::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
  kin_model_->getOrientation(q, link_id, ori);
}

void OC2Model::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
  sejong::Vect3 vel_tmp;
  kin_model_->getVelocity(q, qdot, link_id, vel_tmp);
  sejong::Vect3 ang_vel;
  kin_model_->getAngVel(q, qdot, link_id, ang_vel);
  vel[0] = vel_tmp[0];
  vel[1] = vel_tmp[2];
  vel[2] = ang_vel[1];
}

void OC2Model::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void OC2Model::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void OC2Model::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
    // kin_model_->getCoMPos(q, com_pos, update);
}

void OC2Model::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void OC2Model::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
  // kin_model_->getCentroidVelocity(centroid_vel);
}
