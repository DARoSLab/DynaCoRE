#include "DracoModel.hpp"
#include "Draco_Dyn_Model.hpp"
#include "Draco_Kin_Model.hpp"
#include "rbdl/urdfreader.h"
#include <Utils/utilities.hpp>

#include <stdio.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

DracoModel* DracoModel::GetDracoModel(){
    static DracoModel draco_model_;
    return & draco_model_;
}

DracoModel::DracoModel(){
  model_ = new Model();
  rbdl_check_api_version (RBDL_API_VERSION);

  model_->gravity = Vector3d (0.,0.,  -9.81);

  Body vlink_x, vlink_z, link_body, link_thigh, link_shank, link_foot, link_toe, link_heel;

  Joint vjoint_x = Joint (JointTypePrismatic, Vector3d (1., 0., 0.) );
  Joint vjoint_z = Joint (JointTypePrismatic, Vector3d (0., 0., 1.) );
  Joint joint_ry = Joint (JointTypeRevolute, Vector3d (0., 1., 0.) );

  Matrix3d inertia_body, inertia_upperLeg, inertia_lowerLeg, inertia_foot;
  Vector3d body_com, upperLeg_com, lowerLeg_com, foot_com;
  Vector3d bodyPitch_joff, Knee_joff, Ankle_joff;
  double lx_toe, lx_heel, lz_foot;

  lx_toe = 0.1;
  lx_heel = -0.1;
  lz_foot = -0.03;

  // xx, yy, zz, xy, yz, zx
  inertia_body <<
    0.25, - 0.002, 0.006,
    -0.002, 0.268, 0.001,
    0.006, 0.001, 0.114;
  inertia_upperLeg <<
    0.077, -0.0001, 0.0005,
    -0.0001, 0.0761, 0.0084,
    0.0005, 0.0084, 0.0061;
  inertia_lowerLeg <<
    0.077, -0.0001, -0.0005,
    -0.0001, 0.0761, 0.0084,
    -0.0005, 0.0084, 0.0061;
  inertia_foot <<
    0.001766, -0.0000003, 0.0000006,
    -0.0000003, 0.0020957, -0.0001134,
    0.0000006, -0.0001134, 0.0004916;

  // x, y, z
  body_com << 0., 0., 0.;
  upperLeg_com << 0.0061, 0., -0.2473;
  lowerLeg_com << -0.0038, -0.0022, -0.2647;
  foot_com << 0.0183, 0., -0.0232;

  // joint offset
  bodyPitch_joff << 0.007, -0.007, -0.115;
  Knee_joff << 0.0, 0.0, -0.49;
  Ankle_joff << 0., 0., -0.49;

  double mass_body = 11.313;
  double mass_upperLeg = 5.4818;
  double mass_lowerLeg = 3.7701;
  double mass_foot  = 0.6817573;

  //////////////////////////////////////////////////////
  ///                 Assemble Model                 ///
  //////////////////////////////////////////////////////
  Vector3d com_pos_zero, gyration_radii_zero;
  com_pos_zero.setZero();
  gyration_radii_zero.setZero();
  double mass_zero(1.e-8);

  // ground to Virtual X link
  vlink_x = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlx_id = model_->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), vjoint_x, vlink_x, "virtual_x");
  printf("X virtual id: %i \n", vlx_id);

  // Virtual X link to Virtual Z
  vlink_z = Body (mass_zero, com_pos_zero, gyration_radii_zero);
  int vlz_id = model_->AddBody(vlx_id, Xtrans(Vector3d(0.,0.,0.)), vjoint_z, vlink_z, "virtual_z");
  printf("Z virtual Z id: %i \n", vlz_id);

  // Virtual Z link to Body
  link_body = Body (mass_body, body_com, inertia_body);
  int body_id = model_->AddBody(vlz_id, Xtrans(Vector3d(0., 0., 0.)), joint_ry, link_body, "body");
  printf("body id: %i \n", body_id);

  // Body to Thigh
  link_thigh = Body (mass_upperLeg, upperLeg_com, inertia_upperLeg);
  int thigh_id = model_->AddBody(body_id, Xtrans(bodyPitch_joff),
                                joint_ry, link_thigh, "upperLeg");
  printf("upperLeg id: %i \n", thigh_id);

  // Thigh to Shank
  link_shank = Body (mass_lowerLeg, lowerLeg_com, inertia_lowerLeg);
  int shank_id = model_->AddBody(thigh_id, Xtrans(Knee_joff),
                                joint_ry, link_shank, "lowerLeg");
  printf("lowerLeg id: %i \n", shank_id);

  // Shank to Foot
  link_foot = Body (mass_foot, foot_com, inertia_foot);
  int foot_id = model_->AddBody(shank_id, Xtrans(Ankle_joff),
                               joint_ry, link_foot, "foot");
  printf("foot id: %i \n", foot_id);

  Joint fixed_joint = Joint(JointTypeFixed);
  // Fixed Joint (Toe)
  link_toe = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int toe_id = model_->AddBody(foot_id, Xtrans(Vector3d(lx_toe, 0., lz_foot)), fixed_joint, link_toe, "toe");
  printf("toe id: %i \n", toe_id);

  // Fixed Joint (Heel)
  link_heel = Body(mass_zero, com_pos_zero, gyration_radii_zero);
  int heel_id = model_->AddBody(foot_id, Xtrans(Vector3d(lx_heel, 0., lz_foot)),
                               fixed_joint, link_heel, "heel");
  printf("body heel id: %i \n", heel_id);

  //////////////////////////////////////////////////////
  ///            End of Assemble Model               ///
  //////////////////////////////////////////////////////


    dyn_model_ = new Draco_Dyn_Model(model_);
    kin_model_ = new Draco_Kin_Model(model_);

    printf("[Draco Model] Contructed\n");
}

DracoModel::~DracoModel(){
    delete dyn_model_;
    delete kin_model_;
    delete model_;
}
void DracoModel::UpdateModel(const Vector & q, const Vector & qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
    dyn_model_->UpdateDynamics(q, qdot);
    kin_model_->UpdateKinematics(q, qdot);
}

void DracoModel::getCentroidInertia(sejong::Matrix & Icent){
  sejong::Matrix Icm_tmp;
  kin_model_->getCentroidInertia(Icm_tmp);
  // sejong::pretty_print(Icm_tmp, std::cout, "Icm");
  Icent = Icm_tmp.block(3, 3, 3, 3);
  Icent(2,2) = Icm_tmp(1,1);
}

void DracoModel::getCentroidJacobian(sejong::Matrix & Jcent){
  sejong::Matrix Jcent_tmp(6, NUM_QDOT);
  Jcent_tmp.setZero();
  kin_model_->getCentroidJacobian(Jcent_tmp);

  Jcent = Jcent_tmp.block(3, 0, 3, NUM_QDOT);
  Jcent.block(1,0, 1, NUM_QDOT) = Jcent_tmp.block(5, 0, 1, NUM_QDOT); //Z
  Jcent.block(2,0, 1, NUM_QDOT) = Jcent_tmp.block(1, 0, 1, NUM_QDOT); //Ry
}

void DracoModel::UpdateKinematics(const Vector & q, const Vector &qdot){
    UpdateKinematicsCustom(*model_, &q, &qdot, NULL);
}

bool DracoModel::getInverseMassInertia(sejong::Matrix & Ainv) {
    return dyn_model_->getInverseMassInertia(Ainv);
}

bool DracoModel::getMassInertia(sejong::Matrix & A) {
    return dyn_model_->getMassInertia(A);
}

bool DracoModel::getGravity(Vector & grav) {
    return dyn_model_->getGravity(grav);
}

bool DracoModel::getCoriolis(Vector & coriolis) {
    return dyn_model_->getCoriolis(coriolis);
}

void DracoModel::getFullJacobian(const Vector & q, int link_id, sejong::Matrix & J) const {
  sejong::Matrix Jtmp(6, NUM_QDOT);
  Jtmp.setZero();
  kin_model_->getJacobian(q, link_id, Jtmp);

  // X, Z, Ry
  J = Jtmp.block(3,0, 3, NUM_QDOT);
  J.block(1,0, 1, NUM_QDOT) = Jtmp.block(5, 0, 1, NUM_QDOT); // Z
  J.block(2,0, 1, NUM_QDOT) = Jtmp.block(1, 0, 1, NUM_QDOT); // Ry
}
void DracoModel::getFullJacobianDot(const Vector & q, const Vector & qdot, int link_id, sejong::Matrix & Jdot) const {
  sejong::Matrix Jdot_analytic;
  kin_model_->getJacobianDot6D_Analytic(q, qdot, link_id, Jdot_analytic);

  // X, Z, Ry
  Jdot = Jdot_analytic.block(3,0, 3, NUM_QDOT); // X, Y, Z
  Jdot.block(1,0, 1, NUM_QDOT) = Jdot_analytic.block(5, 0, 1, NUM_QDOT); // Z
  Jdot.block(2,0, 1, NUM_QDOT) = Jdot_analytic.block(1, 0, 1, NUM_QDOT); // Ry
}

void DracoModel::getPosition(const Vector & q,
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

void DracoModel::getVelocity(const Vector & q, const Vector &qdot,
                            int link_id, Vect3 & vel) {
  sejong::Vect3 vel_tmp;
  kin_model_->getVelocity(q, qdot, link_id, vel_tmp);
  sejong::Vect3 ang_vel;
  kin_model_->getAngVel(q, qdot, link_id, ang_vel);
  vel[0] = vel_tmp[0];
  vel[1] = vel_tmp[2];
  vel[2] = ang_vel[1];
}

void DracoModel::getCoMJacobian(const Vector & q, sejong::Matrix & J){
    J = sejong::Matrix::Zero(3, model_->qdot_size);
    kin_model_->getCoMJacobian(q, J);
}

void DracoModel::getCoMPosition(const Vector & q, Vect3 & com_pos, bool update){
  com_pos = kin_model_->com_pos_;
    // kin_model_->getCoMPos(q, com_pos, update);
}

void DracoModel::getCoMVelocity(const Vector & q, const Vector & qdot, Vect3 & com_vel){
    kin_model_->getCoMVel(q, qdot, com_vel);
}
void DracoModel::getCentroidVelocity(sejong::Vector & centroid_vel){
  centroid_vel = kin_model_->centroid_vel_;
  // kin_model_->getCentroidVelocity(centroid_vel);
}
