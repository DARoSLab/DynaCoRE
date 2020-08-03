#include "BodyFootTask.hpp"
#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

#include <Utils/utilities.hpp>

BodyFootTask::BodyFootTask(RobotSystem* robot, int swing_foot):
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

    sp_ = Mercury_StateProvider::getStateProvider();
    Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
    // printf("[BodyFoot Task] Constructed\n");
}

BodyFootTask::~BodyFootTask(){}

bool BodyFootTask::_UpdateCommand(void* pos_des,
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

    // Foot Position
    dynacore::Vect3 foot_pos, foot_vel;
    robot_sys_->getPos(swing_foot_, foot_pos);
    robot_sys_->getLinearVel(swing_foot_, foot_vel);

    for(int i(0); i<3; ++i){
        op_cmd_[i+6] = acc_des[i+6] + Kp_vec_[i+6] * ((*pos_cmd)[i+7] - foot_pos[i]) + Kd_vec_[i+6] * (vel_des[i+6] - foot_vel[i]);
    }

    // dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
    // dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
    // dynacore::pretty_print(foot_pos, std::cout, "foot pos");
    // dynacore::pretty_print(acc_des, std::cout, "acc des");
    // dynacore::pretty_print(vel_des, std::cout, "vel des");

    return true;
}

bool BodyFootTask::_UpdateTaskJacobian(){
    Jt_.block(0,0, 6, 6) = dynacore::Matrix::Identity(6,6);

    dynacore::Matrix Jfoot, Jswing;
    robot_sys_->getFullJacobian(sp_->stance_foot_, Jfoot);
    robot_sys_->getFullJacobian(swing_foot_, Jswing);
    Jt_.block(6, 0, 3, mercury::num_qdot) = 
        Jswing.block(3,0,3, mercury::num_qdot) - Jfoot.block(3, 0, 3, mercury::num_qdot);

    // dynacore::pretty_print(Jswing, std::cout, "Jswing");
    // dynacore::pretty_print(Jfoot, std::cout, "Jfoot");
    // dynacore::pretty_print(Jt_, std::cout, "Jt BodyFoot");
    return true;
}

bool BodyFootTask::_UpdateTaskJDotQdot(){
    // TODO
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);

    dynacore::Matrix Jfoot_dot;
    robot_sys_->getFullJacobianDot(swing_foot_, Jfoot_dot);
    JtDotQdot_.tail(3) = Jfoot_dot.block(3, 0, 3, mercury::num_qdot) * sp_->Qdot_;

    return true;
}

