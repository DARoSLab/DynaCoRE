#include "PitchContact.hpp"
#include <Humanoid/Humanoid_Model.hpp>
#include <Humanoid/Humanoid_Definition.h>
#include <Humanoid_Controller/Humanoid_StateProvider.hpp>

// [ Local Ry, Fx, Fy, Fz ]
PitchContact::PitchContact(const RobotSystem* robot, int pt):
    WBDC_ContactSpec(4),
    contact_pt_(pt),
    max_Fz_(1000.),
    dim_U_(8)
{
  idx_Fz_ = 3;
  robot_sys_ = robot;
  sp_ = Humanoid_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(dim_contact_, humanoid::num_qdot);
}

PitchContact::~PitchContact(){  }

bool PitchContact::_UpdateJc(){
    dynacore::Matrix Jtmp, J_local;
    dynacore::Quaternion quat_tmp;

    robot_sys_->getOri(contact_pt_, quat_tmp);
    Eigen::Matrix3d rot_mtx(quat_tmp);
    dynacore::Matrix Ankle_rot_mt(6,6); Ankle_rot_mt.setZero();
    Ankle_rot_mt.topLeftCorner(3,3) = rot_mtx;
    Ankle_rot_mt.bottomRightCorner(3,3) = rot_mtx;

    robot_sys_->getFullJacobian(contact_pt_, Jtmp);
    J_local = Ankle_rot_mt.transpose() * Jtmp;
    Jc_.block(0,0, 1, humanoid::num_qdot) = J_local.block(1, 0, 1, humanoid::num_qdot);
    Jc_.block(1,0, 3, humanoid::num_qdot) = J_local.block(3, 0, 3, humanoid::num_qdot);

  return true;
}

bool PitchContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool PitchContact::_UpdateUf(){
    double mu(0.3);
    double toe(0.07);
    double heel(0.06);

    Uf_ = dynacore::Matrix::Zero(dim_U_, dim_contact_);
    // Ry(0), Fx(1), Fy(2), Fz(3)

    // Linear
    Uf_(0, 3) = 1.;  // Fz >= 0

    Uf_(1, 1) = 1.; Uf_(1, 3) = mu;
    Uf_(2, 1) = -1.; Uf_(2, 3) = mu;

    Uf_(3, 2) = 1.; Uf_(3, 3) = mu;
    Uf_(4, 2) = -1.; Uf_(4, 3) = mu;

    // Angular (Flip)
    Uf_(5, 0) = -1/toe; Uf_(5, 3) = 1;
    Uf_(6, 0) = 1/heel; Uf_(6, 3) = 1;

    // Upper bound of vertical directional reaction force
    Uf_(7, 3) = -1.;  // -Fz >= -max_Fz_
 
  return true;
}

bool PitchContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(dim_U_);
  ieq_vec_[7] = -max_Fz_;
  return true;
}

