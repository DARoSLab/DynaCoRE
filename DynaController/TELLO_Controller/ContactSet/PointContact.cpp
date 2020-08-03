#include "PointContact.hpp"
#include <TELLO/TELLO_Model.hpp>
#include <TELLO/TELLO_Definition.h>
#include <TELLO_Controller/TELLO_StateProvider.hpp>

// [ Fx, Fy, Fz ]
PointContact::PointContact(const RobotSystem* robot, int pt):WBDC_ContactSpec(3),
  contact_pt_(pt), max_Fz_(1000.),
  dim_U_(6)
{
  idx_Fz_ = 2;
  robot_sys_ = robot;
  sp_ = TELLO_StateProvider::getStateProvider();
}

PointContact::~PointContact(){  }

bool PointContact::_UpdateJc(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobian(contact_pt_, Jtmp);
  Jc_ = Jtmp.block(3, 0, 3, tello::num_qdot);
  pretty_print(Jc_, std::cout, "Point Jc");
  return true;
}
bool PointContact::_UpdateJcDotQdot(){
  dynacore::Vector JcDotQdot_tmp;
  robot_sys_->getFullJDotQdot(contact_pt_, JcDotQdot_tmp);
  JcDotQdot_ = JcDotQdot_tmp.tail(3);
  return true;
}

bool PointContact::_UpdateUf(){
  double mu (0.4);
  Uf_ = dynacore::Matrix::Zero(6, dim_contact_);
  Uf_(0, 2) = 1.; // Fz >= 0

  Uf_(1, 0) = 1.; Uf_(1, 2) = mu; // Fx >= - mu * Fz
  Uf_(2, 0) = -1.; Uf_(2, 2) = mu; // Fx <= mu * Fz

  Uf_(3, 1) = 1.; Uf_(3, 2) = mu; // Fy >= - mu * Fz
  Uf_(4, 1) = -1.; Uf_(4, 2) = mu; // Fy <=  mu * Fz

  Uf_(5, 2) = -1.; // -Fz >=  -z_max
  return true;
}

bool PointContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(6);
  ieq_vec_[5] = -max_Fz_;
  return true;
}
