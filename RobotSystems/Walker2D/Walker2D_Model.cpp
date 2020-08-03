#include "Walker2D_Model.hpp"
#include "Walker2D_Dyn_Model.hpp"
#include "Walker2D_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Utils/utilities.hpp>

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Walker2D_Model* Walker2D_Model::GetWalker2D_Model(){
    static Walker2D_Model walker2d_model_;
    return & walker2d_model_;
}

Walker2D_Model::Walker2D_Model(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);
  model_->gravity = Vector3d (0.,0.,  -9.81);

  Matrix3d inertia;
  Vector3d com_pos, com_pos_body, link_length, link_length_body;
  com_pos.setZero(); link_length.setZero();
  com_pos_body.setZero(); link_length_body.setZero();
  // Joint
  Joint vjoint_x = Joint(JointTypePrismatic, Vector3d (1., 0., 0.)  );
  Joint vjoint_z = Joint(JointTypePrismatic, Vector3d (0., 0., 1.)  );
  Joint joint_ry = Joint (JointTypeRevolute, Vector3d (0., 1., 0.) );
  Joint fixed_joint = Joint(JointTypeFixed);

  Body vlk_x, vlk_z, lk_body, lk_lthigh, lk_lshank, lk_rthigh, lk_rshank, lk_lfoot, lk_rfoot, lk_hip, lk_body_ee, lk_lknee, lk_rknee;

  //////////////////////////////////////////////////////
  ///                   Parameters                   ///
  //////////////////////////////////////////////////////
  double mass (1.7);
  double mass_zero(0.000000001);
  Vector3d com_pos_zero; com_pos_zero.setZero();
  Vector3d gyration_radii_zero; gyration_radii_zero.setZero();

  com_pos[2] = -0.15;
  link_length[2] = -0.3;
  com_pos_body[2] = 0.2;
  link_length_body[2] = 0.37;
  // xx, yy, zz, xy, yz, zx
  inertia <<
    0.001, 0.0, 0.0,
    0.0, 0.2, 0.0,
    0.0, 0.0, 0.2;

  //////////////////////////////////////////////////////
  ///                 Assemble Model                 ///
  //////////////////////////////////////////////////////
  // ground to Virtual X link
  vlk_x = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlx_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), vjoint_x, vlk_x, "virtual_x");
  printf("body X virtual id: %i \n", vlx_id);

  // Virtual X link to Virtual Z link
  vlk_z = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlz_id = model_->AddBody(vlx_id, Xtrans(Vector3d(0., 0., 0.)), vjoint_z, vlk_z, "virtual_z");
  printf("body Z virtual id: %i \n", vlz_id);

  // Virtual Z link to Body
  lk_body = Body (mass, com_pos_body, inertia);
  int body_id = model_->AddBody(vlz_id, Xtrans(Vector3d(0.,0.,0.)), joint_ry, lk_body, "body");
  printf("Body id: %i \n", body_id);

  // Body to Left Thigh
  lk_lthigh = Body (mass, com_pos, inertia);
  int lthigh_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)),
                                  joint_ry, lk_lthigh, "leftThigh");
  printf("Left Thigh id: %i \n", lthigh_id);

  // Left Thigh to Left Shank
  lk_lshank = Body (mass, com_pos, inertia);
  int lshank_id = model_->AddBody(lthigh_id, Xtrans(link_length),
                                  joint_ry, lk_lshank, "leftShank");
  printf("Left Shank id: %i \n", lshank_id);

  // Body to Right Thigh
  lk_rthigh = Body (mass, com_pos, inertia);
  int rthigh_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)),
                                  joint_ry, lk_rthigh, "rightThigh");
  printf("Right Thigh id: %i \n", rthigh_id);

  // Right Thigh to Right Shank
  lk_rshank = Body (mass, com_pos, inertia);
  int rshank_id = model_->AddBody(rthigh_id, Xtrans(link_length),
                                  joint_ry, lk_rshank, "rightShank");
  printf("Right Shank id: %i \n", rshank_id);

  ///////////////////////////////////////////////////////////////
  // Fixed Joint (Hip)
  lk_hip = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int hip_id = model_->AddBody(body_id, Xtrans(Vector3d(0.,0.,0.)), fixed_joint, lk_hip, "hip");
  printf("Hip id: %i \n", hip_id);

  // Fixed Joint (Body EE)
  lk_body_ee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int body_ee_id = model_->AddBody(body_id, Xtrans(link_length_body), fixed_joint, lk_body_ee, "body_ee");
  printf("Body ee id: %i \n", body_ee_id);

  // Fixed Joint (Left Knee)
  lk_lknee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int lknee_id = model_->AddBody(lthigh_id, Xtrans(link_length), fixed_joint, lk_lknee, "leftKnee");
  printf("Left Knee id: %i \n", lknee_id);

  // Fixed Joint (Right Knee)
  lk_rknee = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int rknee_id = model_->AddBody(rthigh_id, Xtrans(link_length), fixed_joint, lk_rknee, "rightKnee");
  printf("Right Knee id: %i \n", rknee_id);

  // Fixed Joint (Left Foot)
  lk_lfoot = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int lfoot_id = model_->AddBody(lshank_id, Xtrans(link_length), fixed_joint, lk_lfoot, "leftFoot");
  printf("Left Foot id: %i \n", lfoot_id);

  // Fixed Joint (Right Foot)
  lk_rfoot = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int rfoot_id = model_->AddBody(rshank_id, Xtrans(link_length), fixed_joint, lk_rfoot, "rightFoot");
  printf("Right Foot id: %i \n", rfoot_id);

  //////////////////////////////////////////////////////
  ///            End of Assemble Model               ///
  //////////////////////////////////////////////////////

  dyn_model_ = new Walker2D_Dyn_Model(model_);
  kin_model_ = new Walker2D_Kin_Model(model_);

  printf("[Walker2D Model] Contructed\n");
}

Walker2D_Model::~Walker2D_Model(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void Walker2D_Model::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void Walker2D_Model::getCentroidInertia(sejong::Matrix & Icent){
  sejong::Matrix Icm_tmp;
  kin_model_->getCentroidInertia(Icm_tmp);
  // sejong::pretty_print(Icm_tmp, std::cout, "Icm");
  Icent = Icm_tmp.block(3, 3, 3, 3);
  Icent(2,2) = Icm_tmp(2,2);
}

void Walker2D_Model::getCentroidJacobian(sejong::Matrix & Jcent){
  sejong::Matrix Jcent_tmp(6, NUM_QDOT);
  Jcent_tmp.setZero();
  kin_model_->getCentroidJacobian(Jcent_tmp);

  Jcent = Jcent_tmp.block(3, 0, 3, NUM_QDOT);
  Jcent.block(1,0, 1, NUM_QDOT) = Jcent_tmp.block(5, 0, 1, NUM_QDOT); //Z
  Jcent.block(2,0, 1, NUM_QDOT) = Jcent_tmp.block(2, 0, 1, NUM_QDOT); //Ry
}

void Walker2D_Model::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool Walker2D_Model::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool Walker2D_Model::getMassInertia(sejong::Matrix & A) {
    return dyn_model_->getMassInertia(A);
}

bool Walker2D_Model::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool Walker2D_Model::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void Walker2D_Model::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  sejong::Matrix Jtmp(6, NUM_QDOT);
  Jtmp.setZero();
  kin_model_->getJacobian(q, link_id, Jtmp);

  // X, Z, Ry
  J = Jtmp.block(3,0, 3, NUM_QDOT);
  J.block(1,0, 1, NUM_QDOT) = Jtmp.block(5, 0, 1, NUM_QDOT); // Z
  J.block(2,0, 1, NUM_QDOT) = Jtmp.block(1, 0, 1, NUM_QDOT); // Ry
}
void Walker2D_Model::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  sejong::Matrix Jdot_analytic;
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot_analytic);

  // X, Z, Ry
  Jdot = Jdot_analytic.block(3,0, 3, NUM_QDOT); // X, Y, Z
  Jdot.block(1,0, 1, NUM_QDOT) = Jdot_analytic.block(5, 0, 1, NUM_QDOT); // Z
  Jdot.block(2,0, 1, NUM_QDOT) = Jdot_analytic.block(1, 0, 1, NUM_QDOT); // Ry
}

void Walker2D_Model::getPosition(const Vector & q,
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
void Walker2D_Model::getOrientation(const Vector & q,
                               int link_id, sejong::Quaternion & ori) {
  kin_model_->getOrientation(q, link_id, ori);
}

void Walker2D_Model::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
  sejong::Vect3 vel_tmp;
  kin_model_->getVelocity(q, qdot, link_id, vel_tmp);
  sejong::Vect3 ang_vel;
  kin_model_->getAngVel(q, qdot, link_id, ang_vel);
  vel[0] = vel_tmp[0];
  vel[1] = vel_tmp[2];
  vel[2] = ang_vel[1];
}

void Walker2D_Model::getAngVel(const Vector & q, const Vector & qdot,
                          int link_id, Vect3 & ang_vel){
    kin_model_->getAngVel(q, qdot, link_id, ang_vel);
}

void Walker2D_Model::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void Walker2D_Model::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
    // kin_model_->getCoMPos(q, com_pos, update);
}

void Walker2D_Model::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void Walker2D_Model::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
  // kin_model_->getCentroidVelocity(centroid_vel);
}
