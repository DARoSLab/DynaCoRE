#include "CoMOriTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

CoMOriTask::CoMOriTask(RobotSystem* robot):WBDC_Relax_Task(6)
{
  Kp_vec_ = dynacore::Vector(dim_task_);
  Kd_vec_ = dynacore::Vector(dim_task_);

  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 100.0;
    Kd_vec_[i] = 10.0;
  }

  sp_ = Mercury_StateProvider::getStateProvider();
  robot_sys_ = robot;
  Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
  // printf("[CoMOri Task] Constructed\n");
}

CoMOriTask::~CoMOriTask(){}

bool CoMOriTask::_UpdateCommand(void* pos_des,
                                    const dynacore::Vector & vel_des,
                                    const dynacore::Vector & acc_des){

  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

  sp_->com_pos_des_ = (*pos_cmd).head(3);
  sp_->com_vel_des_ = (vel_des).head(3);


  dynacore::Vect3 com_pos, com_vel;
  robot_sys_->getCoMPosition(com_pos);
  robot_sys_->getCoMVelocity(com_vel);
  op_cmd_ = dynacore::Vector::Zero(dim_task_);

  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] + Kp_vec_[i] * ((*pos_cmd)[i] - com_pos[i]) + Kd_vec_[i] * (vel_des[i] - com_vel[i]);
  }

  // Orientation
  dynacore::Quaternion curr_quat;
  curr_quat.w() = sp_->Q_[mercury::num_qdot];
  curr_quat.x() = sp_->Q_[3];
  curr_quat.y() = sp_->Q_[4];
  curr_quat.z() = sp_->Q_[5];

  dynacore::Quaternion des_quat;
  sp_->body_ori_des_.w() = (*pos_cmd)[3];
  sp_->body_ori_des_.x() = (*pos_cmd)[4];
  sp_->body_ori_des_.y() = (*pos_cmd)[5];
  sp_->body_ori_des_.z() = (*pos_cmd)[6];
  sp_->body_ang_vel_des_ = (vel_des).tail(3);

  dynacore::Quaternion err_quat = dynacore::QuatMultiply(sp_->body_ori_des_, curr_quat.inverse());

  dynacore::Vect3 ori_err;
  dynacore::convert(err_quat, ori_err);
  // dynacore::pretty_print(err_quat, std::cout, "err quat");
  // dynacore::pretty_print(ori_err, std::cout, "so3 err");

  for(int i(0); i<3; ++i){
    op_cmd_[i+3] = acc_des[i+3] + Kp_vec_[i+3] * ori_err[i] + Kd_vec_[i+3] * (vel_des[i+1] - sp_->Qdot_[i+3]);
  }

  // dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
  // dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
  // dynacore::pretty_print(com_pos, std::cout, "com pos");
  return true;
}

bool CoMOriTask::_UpdateTaskJacobian(){
  dynacore::Matrix Jcom;
  robot_sys_->getCoMJacobian(Jcom);

  Jt_.block(0,0, 3, mercury::num_qdot) = Jcom;
  Jt_(3, 3) = 1.;
  Jt_(4, 4) = 1.;
  Jt_(5, 5) = 1.;
  // dynacore::pretty_print(Jt_, std::cout, "Jt CoMOri");
  return true;
}

bool CoMOriTask::_UpdateTaskJDotQdot(){
  // TODO
  JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
  return true;
}
