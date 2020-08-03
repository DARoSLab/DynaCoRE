#include "BodyOriTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

BodyOriTask::BodyOriTask():Task(6)
{
  Kp_vec_ = dynacore::Vector(dim_task_);
  Kd_vec_ = dynacore::Vector(dim_task_);

  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 100.0;
    Kd_vec_[i] = 10.0;
  }

  sp_ = Mercury_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
  // printf("[BodyOri Task] Constructed\n");
}

BodyOriTask::~BodyOriTask(){}

bool BodyOriTask::_UpdateCommand(void* pos_des,
                                    const dynacore::Vector & vel_des,
                                    const dynacore::Vector & acc_des){

  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
  op_cmd_ = dynacore::Vector::Zero(dim_task_);

  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] + 
        Kp_vec_[i] * ((*pos_cmd)[i] - sp_->Q_[i]) + 
        Kd_vec_[i] * (vel_des[i] - sp_->Qdot_[i]);

    sp_->body_pos_des_[i] = (*pos_cmd)[i];
    sp_->body_vel_des_[i] = vel_des[i];
    sp_->body_pos_[i] = sp_->Q_[i];
    sp_->body_vel_[i] = sp_->Qdot_[i];
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

bool BodyOriTask::_UpdateTaskJacobian(){
  Jt_.block(0,0, 6, 6) = dynacore::Matrix::Identity(6,6);
  // dynacore::pretty_print(Jt_, std::cout, "Jt BodyOri");
  return true;
}

bool BodyOriTask::_UpdateTaskJDotQdot(){
  JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
  return true;
}
