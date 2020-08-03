#include "StanceTask.hpp"
// Task consist of virtual joint (6) and stance leg joint (3)

#include <Configuration.h>
#include <Mercury_Controller/Mercury_StateProvider.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Utils/utilities.hpp>

StanceTask::StanceTask(int stance_leg):Task(9),
    Kp_vec_(9),
    Kd_vec_(9),
    stance_leg_(stance_leg)
{
    if(stance_leg_ == mercury_link::leftFoot){
        stance_leg_jidx_ = mercury_joint::leftAbduction;
    }else if(stance_leg_ == mercury_link::rightFoot){
        stance_leg_jidx_ = mercury_joint::rightAbduction;
    }else{
        printf("[Stnace Task] Incorrect stance leg\n");
    }

    Kp_vec_.setZero();
    Kd_vec_.setZero();
    for(int i(0); i<dim_task_; ++i){
        Kp_vec_[i] = 150.;
        Kd_vec_[i] = 3.;
    }
    for(int i(0); i<mercury::num_virtual; ++i){
        Kp_vec_[i] = 0.;
        Kd_vec_[i] = 0.;
    }
    sp_ = Mercury_StateProvider::getStateProvider();
    Jt_ = dynacore::Matrix::Zero(dim_task_, mercury::num_qdot);
    JtDotQdot_ = dynacore::Vector::Zero(dim_task_);
}

StanceTask::~StanceTask(){}

bool StanceTask::_UpdateCommand(void* pos_des,
        const dynacore::Vector & vel_des,
        const dynacore::Vector & acc_des){
    dynacore::Vector* pos_cmd = (dynacore::Vector*)pos_des;

    // TODO: Implemented based on the assumption that 
    // we do not control orientation
    for(int i(0); i<mercury::num_virtual; ++i){
        op_cmd_[i] = acc_des[i] 
            + Kp_vec_[i] * ((*pos_cmd)[i] - sp_->Q_[i]) 
            + Kd_vec_[i] * (vel_des[i] - sp_->Qdot_[i]);
    }

    for(int i(mercury::num_virtual); i<mercury::num_virtual + 3; ++i){
        op_cmd_[i] = acc_des[i]
            + Kp_vec_[i] * ((*pos_cmd)[i] 
                    - sp_->Q_[i - mercury::num_virtual + stance_leg_jidx_])
            + Kd_vec_[i] * (vel_des[i] 
                    - sp_->Qdot_[i - mercury::num_virtual + stance_leg_jidx_]);
    }
    //printf("[Stance Task]\n");
     //dynacore::pretty_print(Kp_vec_, std::cout, "Kp");
     //dynacore::pretty_print(Kd_vec_, std::cout, "Kd");
     //dynacore::pretty_print(acc_des, std::cout, "acc_des");
     //dynacore::pretty_print(op_cmd_, std::cout, "op cmd");
     //dynacore::pretty_print(*pos_cmd, std::cout, "pos cmd");
     //dynacore::pretty_print(sp_->Q_, std::cout, "config");
     //dynacore::pretty_print(Jt_, std::cout, "Jt");

    return true;
}

bool StanceTask::_UpdateTaskJacobian(){
    Jt_.block(0,0, mercury::num_virtual, mercury::num_virtual) = 
        dynacore::Matrix::Identity(mercury::num_virtual, mercury::num_virtual);

    Jt_.block(mercury::num_virtual, stance_leg_jidx_, 3, 3) =
        dynacore::Matrix::Identity(3,3);
    return true;
}

bool StanceTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}
