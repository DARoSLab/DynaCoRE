#include "CoMFootJPosTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

CoMFootJPosTask::CoMFootJPosTask(RobotSystem* robot, int swing_foot):
    Task(9),
    robot_sys_(robot),
    swing_foot_(swing_foot)
{
  Kp_vec_ = dynacore::Vector(dim_task_);
  Kd_vec_ = dynacore::Vector(dim_task_);

  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 100.0;
    Kd_vec_[i] = 10.0;
  }

  if (swing_foot_ == mercury_link::rightFoot){
      swing_leg_jidx_ = mercury_joint::rightAbduction;
  }else if(swing_foot_ == mercury_link::leftFoot){
    swing_leg_jidx_ = mercury_joint::leftAbduction;
  }else {
      printf("[CoM Swing Foot JPos Task] Not valid swing foot option\n");
      exit(0);
  }

  sp_ = Mercury_StateProvider::getStateProvider();
  Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
   printf("[CoM Swing Foot JPos Task] Constructed\n");
}

CoMFootJPosTask::~CoMFootJPosTask(){}

bool CoMFootJPosTask::_UpdateCommand(void* pos_des,
                                    const dynacore::Vector & vel_des,
                                    const dynacore::Vector & acc_des){

  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
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
  des_quat.w() = (*pos_cmd)[3];
  des_quat.x() = (*pos_cmd)[4];
  des_quat.y() = (*pos_cmd)[5];
  des_quat.z() = (*pos_cmd)[6];

  dynacore::Quaternion err_quat = dynacore::QuatMultiply(des_quat, curr_quat.inverse());

  dynacore::Vect3 ori_err;
  dynacore::convert(err_quat, ori_err);
  // dynacore::pretty_print(err_quat, std::cout, "err quat");
  // dynacore::pretty_print(ori_err, std::cout, "so3 err");

  for(int i(0); i<3; ++i){
    op_cmd_[i+3] = acc_des[i+3] + Kp_vec_[i+3] * ori_err[i] + Kd_vec_[i+3] * (vel_des[i+1] - sp_->Qdot_[i+3]);
  }

  // Swing Foot JPos 
  for(int i(0); i<3; ++i){
    op_cmd_[i+6] = acc_des[i+6] + 
        Kp_vec_[i+6] * ((*pos_cmd)[i+7] - sp_->Q_[i + swing_leg_jidx_]) + 
        Kd_vec_[i+6] * (vel_des[i+6] - sp_->Qdot_[i + swing_leg_jidx_]);
  }

   //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(foot_pos, std::cout, "foot pos");
   //dynacore::pretty_print(acc_des, std::cout, "acc des");
   //dynacore::pretty_print(vel_des, std::cout, "vel des");

  return true;
}

bool CoMFootJPosTask::_UpdateTaskJacobian(){
  dynacore::Matrix Jbody, Jcom, Jfoot;

  robot_sys_->getCoMJacobian(Jcom);
  robot_sys_->getFullJacobian(sp_->stance_foot_, Jfoot);

  // TODO
  Jt_.block(0,0, 3, mercury::num_qdot) = Jcom - Jfoot.block(3, 0, 3, mercury::num_qdot);
  Jt_(3, 3) = 1.;
  Jt_(4, 4) = 1.;
  Jt_(5, 5) = 1.;

  Jt_.block(6, swing_leg_jidx_, 3, 3) = dynacore::Matrix::Identity(3,3);

  // dynacore::pretty_print(Jswing, std::cout, "Jswing");
  // dynacore::pretty_print(Jfoot, std::cout, "Jfoot");
   //dynacore::pretty_print(Jt_, std::cout, "Jt CoMFootJPos");
  return true;
}

bool CoMFootJPosTask::_UpdateTaskJDotQdot(){
  JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
  return true;
}

