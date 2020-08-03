#include "LinkXYZTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/utilities.hpp>

LinkXYZTask::LinkXYZTask(RobotSystem* robot, int link_id):WBDC_Relax_Task(3)
{
  link_id_ = link_id;
  Kp_vec_ = dynacore::Vector(dim_task_);
  Kd_vec_ = dynacore::Vector(dim_task_);

  for(int i(0); i<dim_task_; ++i){
    Kp_vec_[i] = 100.0;
    Kd_vec_[i] = 10.0;
  }

  sp_ = Mercury_StateProvider::getStateProvider();
  robot_sys_ = robot;
  Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
  // printf("[LinkXYZTask Task] Constructed\n");
}

LinkXYZTask::~LinkXYZTask(){}

bool LinkXYZTask::_UpdateCommand(void* pos_des,
                                    const dynacore::Vector & vel_des,
                                    const dynacore::Vector & acc_des){

  dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;
  dynacore::Vect3 pos, vel;
  robot_sys_->getPos(link_id_, pos);
  robot_sys_->getLinearVel(link_id_, vel);
  op_cmd_ = dynacore::Vector::Zero(dim_task_);

  for(int i(0); i<3; ++i){
    op_cmd_[i] = acc_des[i] + Kp_vec_[i] * ((*pos_cmd)[i] - pos[i]) + Kd_vec_[i] * (vel_des[i] - vel[i]);
  }

   //dynacore::pretty_print(acc_des, std::cout, "acc_des");
   //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
   //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
   //dynacore::pretty_print(pos, std::cout, "pos");

  return true;
}

bool LinkXYZTask::_UpdateTaskJacobian(){
  dynacore::Matrix Jt_full;
  robot_sys_->getFullJacobian(link_id_, Jt_full);
  Jt_ = Jt_full.block(3, 0, 3, mercury::num_qdot);
  return true;
}

bool LinkXYZTask::_UpdateTaskJDotQdot(){
  JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
  dynacore::Matrix Jdot;
  robot_sys_->getFullJacobianDot(link_id_, Jdot);
  dynacore::Vector JtdotQdot_full= Jdot * sp_->Qdot_;
  JtDotQdot_ = JtdotQdot_full.tail(3);

  // dynacore::pretty_print(JtDotQdot_, std::cout, "JtDot Qdot");
  return true;
}

