#include "SingleContact.hpp"
#include <Cheetah3/Cheetah3_Model.hpp>
#include <Cheetah3/Cheetah3_Definition.h>
#include <Cheetah3_Controller/Cheetah3_StateProvider.hpp>

// [ Tau_x, Tau_y, Tau_z, Rx, Ry, Rz ]
SingleContact::SingleContact(const RobotSystem* robot, int pt):
    WBDC_ContactSpec(3),
    contact_pt_(pt),
    max_Fz_(500.),
    dim_U_(5)
{
    idx_Fz_ = 2;
  robot_sys_ = robot;
  sp_ = Cheetah3_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(dim_contact_, cheetah3::num_qdot);
}

SingleContact::~SingleContact(){  }

bool SingleContact::_UpdateJc(){
  robot_sys_->getFullJacobian(contact_pt_, Jc_);
  return true;
}

bool SingleContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool SingleContact::_UpdateUf(){
  double mu (0.3);
  Uf_ = dynacore::Matrix::Zero(dim_U_, dim_contact_);

  Uf_(0, 2) = 1.;

  Uf_(1, 0) = 1.; Uf_(1, 2) = mu;
  Uf_(2, 0) = -1.; Uf_(2, 2) = mu;

  Uf_(3, 1) = 1.; Uf_(3, 2) = mu;
  Uf_(4, 1) = -1.; Uf_(4, 2) = mu;

  // ////////////////////////////////////////////////////
  Uf_(5, 2) = -1.;

  return true;
}

bool SingleContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(dim_U_);
  ieq_vec_[5] = -max_Fz_;
  return true;
}

