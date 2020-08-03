#include "DoubleContact.hpp"
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Utils/utilities.hpp>

DoubleContact::DoubleContact(RobotSystem* robot):WBDC_ContactSpec(6)
{
  robot_sys_ = robot;
  sp_ = Mercury_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix(6, mercury::num_qdot);
  // printf("[Double Contact] Constructed\n");
}

DoubleContact::~DoubleContact(){}

bool DoubleContact::_UpdateJc(){
  dynacore::Matrix Jtmp;
  robot_sys_->getFullJacobian(mercury_link::rightFoot, Jtmp);
  Jc_.block(0, 0, 3, mercury::num_qdot) = Jtmp.block(3, 0, 3, mercury::num_qdot);

  robot_sys_->getFullJacobian(mercury_link::leftFoot, Jtmp);
  Jc_.block(3, 0, 3, mercury::num_qdot) = Jtmp.block(3, 0, 3, mercury::num_qdot);

  // dynacore::pretty_print(Jc_, std::cout, "double] Jc");
  return true;
}
bool DoubleContact::_UpdateJcDotQdot(){
  dynacore::Matrix JcDot(dim_contact_, mercury::num_qdot);
  dynacore::Matrix jcdot_tmp;
  // Right
  robot_sys_->getFullJacobianDot(mercury_link::rightFoot, jcdot_tmp);
  JcDot.block(0, 0, 3, mercury::num_qdot) = jcdot_tmp.block(3, 0, 3, mercury::num_qdot);
  // Left
  robot_sys_->getFullJacobianDot(mercury_link::leftFoot, jcdot_tmp);
  JcDot.block(3, 0, 3, mercury::num_qdot) = jcdot_tmp.block(3, 0, 3, mercury::num_qdot);

  // dynacore::pretty_print(JcDot, std::cout,  "JcDot");
  JcDotQdot_ = JcDot * sp_->Qdot_;
  return true;
}

bool DoubleContact::_UpdateUf(){
  double mu(0.3);

  int size_u(5);
  Uf_ = dynacore::Matrix::Zero(size_u*2, dim_contact_);

  dynacore::Matrix U;
  _setU(mu, U);
  Uf_.block(0, 0, size_u, 3) = U;
  Uf_.block(size_u, 3, size_u, 3) = U;
  return true;
}

bool DoubleContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(5*2);
  return true;
}

void DoubleContact::_setU(double mu, dynacore::Matrix & U){
  U = dynacore::Matrix::Zero(5, 3);

  U(0, 2) = 1.;

  U(1, 0) = 1.; U(1, 2) = mu;
  U(2, 0) = -1.; U(2, 2) = mu;

  U(3, 1) = 1.; U(3, 2) = mu;
  U(4, 1) = -1.; U(4, 2) = mu;
}
