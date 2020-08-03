#include "FixedBodyContact.hpp"
#include <Cheetah3/Cheetah3_Model.hpp>
#include <Cheetah3/Cheetah3_Definition.h>
#include <Cheetah3_Controller/Cheetah3_StateProvider.hpp>
#include <Utils/utilities.hpp>

FixedBodyContact::FixedBodyContact(RobotSystem* robot):WBDC_ContactSpec(6)
{
  robot_sys_ = robot;
  sp_ = Cheetah3_StateProvider::getStateProvider();
  Jc_ = dynacore::Matrix::Zero(dim_contact_, cheetah3::num_qdot);
}
FixedBodyContact::~FixedBodyContact(){ }

bool FixedBodyContact::_UpdateJc(){
  for(int i(0);i<dim_contact_; ++i)  Jc_(i,i) = 1.;
  return true;
}

bool FixedBodyContact::_UpdateJcDotQdot(){
  JcDotQdot_ = dynacore::Vector::Zero(dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateUf(){
  Uf_ = dynacore::Matrix::Zero(1, dim_contact_);
  return true;
}

bool FixedBodyContact::_UpdateInequalityVector(){
  ieq_vec_ = dynacore::Vector::Zero(1);
  return true;
}
