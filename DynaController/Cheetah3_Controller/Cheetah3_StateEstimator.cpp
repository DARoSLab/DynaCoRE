#include "Cheetah3_StateEstimator.hpp"
#include "Cheetah3_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Cheetah3/Cheetah3_Model.hpp>
#include <Cheetah3_Controller/Cheetah3_DynaCtrl_Definition.h>

// Orientation Estimators
#include <Cheetah3_Controller/StateEstimator/BasicAccumulation.hpp>

Cheetah3_StateEstimator::Cheetah3_StateEstimator(RobotSystem* robot):
    curr_config_(cheetah3::num_q),
    curr_qdot_(cheetah3::num_qdot)
{
    sp_ = Cheetah3_StateProvider::getStateProvider();
    robot_sys_ = robot;
    ori_est_ = new BasicAccumulation();
}

Cheetah3_StateEstimator::~Cheetah3_StateEstimator(){
    delete ori_est_;
}

void Cheetah3_StateEstimator::Initialization(Cheetah3_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[cheetah3::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<cheetah3::num_act_joint; ++i){
        curr_config_[cheetah3::num_virtual + i] = data->jpos[i];
        curr_qdot_[cheetah3::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->EstimatorInitialization(imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[cheetah3::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];

    robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;
    sp_->jpos_ini_ = 
        curr_config_.segment(cheetah3::num_virtual, cheetah3::num_act_joint);
}

void Cheetah3_StateEstimator::Update(Cheetah3_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[cheetah3::num_qdot] = 1.;

    for (int i(0); i<cheetah3::num_act_joint; ++i){
        curr_config_[cheetah3::num_virtual + i] = data->jpos[i];
        curr_qdot_[cheetah3::num_virtual + i] = data->jvel[i];
    }
    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
 
    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }
   
    dynacore::Quaternion body_ori;
    body_ori.w() = 1.; body_ori.x() = 0.; body_ori.y() = 0.; body_ori.z() = 0;
    dynacore::Vect3 body_ang_vel;

    ori_est_->setSensorData( imu_acc, imu_ang_vel);
    ori_est_->getEstimatedState(body_ori, body_ang_vel);

    curr_config_[3] = body_ori.x();
    curr_config_[4] = body_ori.y();
    curr_config_[5] = body_ori.z();
    curr_config_[cheetah3::num_qdot] = body_ori.w();

    for(int i(0); i<3; ++i)  curr_qdot_[i+3] = body_ang_vel[i];
    
    robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

}
