#include "LinkOriTask.hpp"
// (Rx, Ry, Rz)

#include <Configuration.h>
#include <TELLO/TELLO_Definition.h>
#include <Utils/utilities.hpp>
#include <TELLO/TELLO_Model.hpp>
#include <TELLO_Controller/TELLO_StateProvider.hpp>

LinkOriTask::LinkOriTask(const RobotSystem* robot, int link_idx):KinTask(3),
    robot_sys_(robot),
    link_idx_(link_idx)
{
    Jt_ = dynacore::Matrix::Zero(dim_task_, tello::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

LinkOriTask::~LinkOriTask(){}

bool LinkOriTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Quaternion* ori_cmd = (dynacore::Quaternion*)pos_des;
    dynacore::Quaternion link_ori;
    robot_sys_->getOri(link_idx_, link_ori);

    dynacore::Quaternion ori_err = dynacore::QuatMultiply(*ori_cmd, link_ori.inverse());
    dynacore::Vect3 ori_err_so3;
    dynacore::convert(ori_err, ori_err_so3);

    // Rx, Ry, Rz
    for(int i(0); i<3; ++i){
        pos_err_[i] = ori_err_so3[i];
        vel_des_[i] = vel_des[i];
        acc_des_[i] = acc_des[i];
    }
    //printf("[Link Ori Task]\n");
    //dynacore::pretty_print(acc_des, std::cout, "acc_des");
    //dynacore::pretty_print(pos_err_, std::cout, "pos_err_");
    //dynacore::pretty_print(*ori_cmd, std::cout, "ori cmd");
    //dynacore::pretty_print(link_ori, std::cout, "link ori");
    //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool LinkOriTask::_UpdateTaskJacobian(){
    dynacore::Matrix Jtmp;
    robot_sys_->getFullJacobian(link_idx_, Jtmp);
    // Rx, Ry, Rz
    Jt_ = Jtmp.block(0,0, 3, tello::num_qdot);

    return true;
}

bool LinkOriTask::_UpdateTaskJDotQdot(){
    dynacore::Vector tmp;
    robot_sys_->getFullJDotQdot(link_idx_, tmp);
    JtDotQdot_ = tmp.head(3);
    return true;
}
