#include "SingleContact.hpp"
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>

SingleContact::SingleContact(const RobotSystem* robot, int pt):WBDC_ContactSpec(3),
                                     contact_pt_(pt)
{
  robot_sys_ = robot;
  sp_ = Mercury_StateProvider::getStateProvider();
}

SingleContact::~SingleContact(){
}
bool SingleContact::_UpdateJc(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobian(contact_pt_, Jtmp);
  Jc_ = Jtmp.block(3, 0, 3, mercury::num_qdot);
  return true;
}
bool SingleContact::_UpdateJcDotQdot(){
  dynacore::Matrix JcDot;
  robot_sys_->getFullJacobianDot(contact_pt_, JcDot);
  JcDotQdot_ = JcDot.block(3, 0, 3, mercury::num_qdot) * sp_->Qdot_;
  JcDotQdot_.setZero();
  return true;

}

bool SingleContact::_UpdateUf(){
  double mu (0.3);
  Uf_ = dynacore::Matrix::Zero(5, dim_contact_);
  Uf_(0, 2) = 1.; // Fz >= 0

  Uf_(1, 0) = 1.; Uf_(1, 2) = mu; // Fx >= - mu * Fz
  Uf_(2, 0) = -1.; Uf_(2, 2) = mu; // Fx <= mu * Fz

  Uf_(3, 1) = 1.; Uf_(3, 2) = mu; // Fy >= - mu * Fz
  Uf_(4, 1) = -1.; Uf_(4, 2) = mu; // Fy <=  mu * Fz
  return true;
}

bool SingleContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(5);
  return true;
}
