#include "PitchYawContact.hpp"
#include <TELLO/TELLO_Model.hpp>
#include <TELLO/TELLO_Definition.h>
#include <TELLO_Controller/TELLO_StateProvider.hpp>

// [ Local Ry, Local Rz, Fx, Fy, Fz ]
PitchYawContact::PitchYawContact(const RobotSystem* robot, int pt):
    WBDC_ContactSpec(5),
    contact_pt_(pt),
    max_Fz_(1000.),
    dim_U_(12)
{
  idx_Fz_ = 4;
  robot_sys_ = robot;
  sp_ = TELLO_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(dim_contact_, tello::num_qdot);
}

PitchYawContact::~PitchYawContact(){  }

bool PitchYawContact::_UpdateJc(){
    dynacore::Matrix Jtmp, J_local;
    dynacore::Quaternion quat_tmp;

    robot_sys_->getOri(contact_pt_, quat_tmp);
    Eigen::Matrix3d rot_mtx(quat_tmp);
    dynacore::Matrix Ankle_rot_mt(6,6); Ankle_rot_mt.setZero();
    Ankle_rot_mt.topLeftCorner(3,3) = rot_mtx;
    Ankle_rot_mt.bottomRightCorner(3,3) = rot_mtx;

    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    J_local = Ankle_rot_mt.transpose() * Jtmp;
    Jc_.block(0,0, 2, tello::num_qdot) = J_local.block(1, 0, 2, tello::num_qdot);
    Jc_.block(2,0, 3, tello::num_qdot) = J_local.block(3, 0, 3, tello::num_qdot);

  return true;
}

bool PitchYawContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool PitchYawContact::_UpdateUf(){
    double mu(0.4);
    double toe(0.04);
    double heel(0.03);

    Uf_ = dynacore::Matrix::Zero(dim_U_, dim_contact_);
    // Ry(0) Rz(1), Fx(1), Fy(2), Fz(3)

    // Linear
    Uf_(0, 4) = 1.;

    Uf_(1, 2) = 1.; Uf_(1, 4) = mu;
    Uf_(2, 2) = -1.; Uf_(2, 4) = mu;

    Uf_(3, 3) = 1.; Uf_(3, 4) = mu;
    Uf_(4, 3) = -1.; Uf_(4, 4) = mu;

    // Angular (Flip)
    Uf_(5, 0) = -1/toe; Uf_(5, 4) = 1;
    Uf_(6, 0) = 1/heel; Uf_(6, 4) = 1;

    // Yaw
    Uf_(7, 0) = -mu/toe; Uf_(7, 1) = -1/toe; Uf_(7, 3) = -1; Uf_(7, 4) = mu;
    Uf_(8, 0) = -mu/toe; Uf_(8, 1) = 1/toe; Uf_(8, 3) = 1; Uf_(8, 4) = mu;

    Uf_(9, 0) = mu/heel; Uf_(9, 1) = 1/heel; Uf_(9, 3) = -1; Uf_(9, 4) = mu;
    Uf_(10, 0) = mu/heel; Uf_(10, 1) = -1/heel; Uf_(10, 3) = 1; Uf_(10, 4) = mu;

    // Upper bound of vertical directional reaction force
    Uf_(11, 4) = -1.;  // -Fz >= -max_Fz_

    return true;
}

bool PitchYawContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(dim_U_);
  ieq_vec_[11] = -max_Fz_;
  return true;
}

