#include "Mercury_StateEstimator.hpp"
#include "Mercury_StateProvider.hpp"
#include "Mercury_interface.hpp"
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Filter/filters.hpp>

// Mocap based Estimator
#include <Mercury_Controller/StateEstimator/BodyFootPosEstimator.hpp>

// Orientation Estimators
#include <Mercury_Controller/StateEstimator/OriEstAccObs.hpp>
#include <Mercury_Controller/StateEstimator/BasicAccumulation.hpp>
#include <Mercury_Controller/StateEstimator/NoBias.hpp>
#include <Mercury_Controller/StateEstimator/NoAccState.hpp>

// Velocity Estimators
#include <Mercury_Controller/StateEstimator/BiasCompensatedBodyVelocityEstimator.hpp>
#include <Mercury_Controller/StateEstimator/SimpleAverageEstimator.hpp>

// EKF based estimators
#include <Mercury_Controller/StateEstimator/EKF_RotellaEstimator.hpp> // EKF
#include <Mercury_Controller/StateEstimator/EKF_LIPRotellaEstimator.hpp> // EKF

#include "MoCapManager.hpp"

Mercury_StateEstimator::Mercury_StateEstimator(RobotSystem* robot):
    base_cond_(0),
    b_using_jpos_(false),
    curr_config_(mercury::num_q),
    curr_qdot_(mercury::num_qdot),
    jjpos_config_(mercury::num_q),
    jjvel_qdot_(mercury::num_qdot)
{

    sp_ = Mercury_StateProvider::getStateProvider();
    robot_sys_ = robot;

    body_foot_est_ = new BodyFootPosEstimator(robot);
    ori_est_ = new BasicAccumulation();
    //    ekf_est_ = new EKF_RotellaEstimator(); // EKF
    //ekf_est_ = new EKF_LIPRotellaEstimator(); // EKF with Pendulum Dynamics
    bias_vel_est_ = new BiasCompensatedBodyVelocityEstimator();
    vel_est_ = new SimpleAverageEstimator();
    mocap_vel_est_ = new SimpleAverageEstimator();

    for(int i(0); i<2; ++i){
        filter_com_vel_.push_back(
                new digital_lp_filter(2.*M_PI * 5., mercury::servo_rate));
    }

    for(int i(0); i < mercury::num_act_joint; i++){
        filter_jpos_vel_.push_back(
                new deriv_lp_filter(2.0*M_PI*10.0, mercury::servo_rate));
    }

    for(int i(0); i<3; i++){
        filter_ang_vel_.push_back(
                new digital_lp_filter(2.0*M_PI*10.0, mercury::servo_rate));
    }
}

Mercury_StateEstimator::~Mercury_StateEstimator(){
    delete body_foot_est_;
    delete bias_vel_est_;
    delete vel_est_;
    delete mocap_vel_est_;
}

void Mercury_StateEstimator::Initialization(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;

    jjpos_config_.setZero();
    jjvel_qdot_.setZero();
    jjpos_config_[mercury::num_qdot] = 1.;

    // Joint Set
    for (int i(0); i<mercury::num_act_joint; ++i){
        if(b_using_jpos_){
            curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];    
        } else{
            curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        }
          
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];
        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];
        // Joint encoder update
        jjpos_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        jjvel_qdot_[mercury::num_virtual + i] = data->joint_jvel[i];
        sp_->mjpos_[i] = data->motor_jpos[i];
    }

    // TEST
    // curr_config_[mercury_joint::rightKnee] = 
    //    data->joint_jpos[mercury_joint::rightKnee - mercury::num_virtual];
    // curr_config_[mercury_joint::leftKnee] = 
    //    data->joint_jpos[mercury_joint::leftKnee - mercury::num_virtual];

    // curr_config_[mercury_joint::rightHip] = 
    //    data->joint_jpos[mercury_joint::rightHip - mercury::num_virtual];
    // curr_config_[mercury_joint::leftHip] = 
    //    data->joint_jpos[mercury_joint::leftHip - mercury::num_virtual];

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);

    for(int i(0); i<3; ++i){
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
    }

    ori_est_->EstimatorInitialization(sp_->body_ori_, imu_acc, imu_ang_vel);   
    ori_est_->getEstimatedState(sp_->body_ori_, sp_->body_ang_vel_);
    //ekf_est_->EstimatorInitialization(sp_->body_ori_, imu_acc, imu_ang_vel); // EKF
    bias_vel_est_->EstimatorInitialization(imu_acc, sp_->body_ori_);
    body_foot_est_->Initialization(sp_->body_ori_);

    // Local Frame Setting
    if(base_cond_ == base_condition::floating){
        curr_config_[3] = sp_->body_ori_.x();
        curr_config_[4] = sp_->body_ori_.y();
        curr_config_[5] = sp_->body_ori_.z();
        curr_config_[mercury::num_qdot] = sp_->body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = sp_->body_ang_vel_[i];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        dynacore::Vect3 foot_pos, foot_vel;
        robot_sys_->getPos(sp_->stance_foot_, foot_pos);
        robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
        curr_config_[0] = -foot_pos[0];
        curr_config_[1] = -foot_pos[1];
        curr_config_[2] = -foot_pos[2];
        curr_qdot_[0] = -foot_vel[0];
        curr_qdot_[1] = -foot_vel[1];
        curr_qdot_[2] = -foot_vel[2];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        /// Jpos based model update  ////////////////////////////////
        if(b_jpos_model_update_){
            jjpos_config_[3] = sp_->body_ori_.x();
            jjpos_config_[4] = sp_->body_ori_.y();
            jjpos_config_[5] = sp_->body_ori_.z();
            jjpos_config_[mercury::num_qdot] = sp_->body_ori_.w();
            for(int i(0); i<3; ++i)
                jjvel_qdot_[i+3] = sp_->body_ang_vel_[i];

            sp_->jjpos_robot_sys_->UpdateSystem(jjpos_config_, jjvel_qdot_);

            sp_->jjpos_robot_sys_->getPos(sp_->stance_foot_, foot_pos);
            sp_->jjpos_robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
            jjpos_config_[0] = -foot_pos[0];
            jjpos_config_[1] = -foot_pos[1];
            jjpos_config_[2] = -foot_pos[2];
            jjvel_qdot_[0] = -foot_vel[0];
            jjvel_qdot_[1] = -foot_vel[1];
            jjvel_qdot_[2] = -foot_vel[2];

            sp_->jjpos_robot_sys_->UpdateSystem(jjpos_config_, jjvel_qdot_);
        }
        /// END of Jpos based model update ///////////////////////////
    } else if (base_cond_ == base_condition::fixed){
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    } else if (base_cond_ == base_condition::lying){
        // pitch rotation (PI/2)
        curr_config_[4] = sin(M_PI/2.0/2.0);
        curr_config_[mercury::num_qdot] = cos(M_PI/2.0/2.0);

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    }
    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);
    ((BasicAccumulation*)ori_est_)->CoMStateInitialization(sp_->CoM_pos_, sp_->CoM_vel_);
    bias_vel_est_->CoMStateInitialization(sp_->CoM_pos_, sp_->CoM_vel_);
    vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);

    // Warning: state provider setup
    // LED data save  ////////////////////
    dynacore::Vect3 pos;
    int led_idx = mercury_link::LED_BODY_0;
    for(int i(0); i<NUM_MARKERS; ++i){
        robot_sys_->getPos(mercury_link::LED_BODY_0 + i, pos);
        for (int j(0); j<3; ++j)  sp_->led_kin_data_[3*i + j] = pos[j];
    }
    //////////////////////////////////////

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    // Foot pos
    robot_sys_->getPos(mercury_link::rightFoot, sp_->Rfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::rightFoot, sp_->Rfoot_vel_);
    robot_sys_->getPos(mercury_link::leftFoot, sp_->Lfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::leftFoot, sp_->Lfoot_vel_);
    for (int i(0); i<3; ++i){
        sp_->jjpos_body_pos_[i] = jjpos_config_[i];
        sp_->jjvel_body_vel_[i] = jjvel_qdot_[i];        
    }
    sp_->SaveCurrentData();
}

void Mercury_StateEstimator::Update(Mercury_SensorData* data){
    curr_config_.setZero();
    curr_qdot_.setZero();
    curr_config_[mercury::num_qdot] = 1.;

    jjpos_config_.setZero();
    jjvel_qdot_.setZero();
    jjpos_config_[mercury::num_qdot] = 1.;

    for (int i(0); i<mercury::num_act_joint; ++i){
        if(b_using_jpos_){
            curr_config_[mercury::num_virtual + i] = data->joint_jpos[i];    
        } else{
            curr_config_[mercury::num_virtual + i] = data->motor_jpos[i];
        }
        curr_qdot_[mercury::num_virtual + i] = data->motor_jvel[i];
        sp_->rotor_inertia_[i] = data->reflected_rotor_inertia[i];

        // Joint encoder update
        jjpos_config_[mercury::num_virtual + i] = data->joint_jpos[i];
        jjvel_qdot_[mercury::num_virtual + i] = data->joint_jvel[i];
        sp_->mjpos_[i] = data->motor_jpos[i];
    }
    
    // TEST
    // curr_config_[mercury_joint::rightKnee] = 
    //    data->joint_jpos[mercury_joint::rightKnee - mercury::num_virtual];
    // curr_config_[mercury_joint::leftKnee] = 
    //    data->joint_jpos[mercury_joint::leftKnee - mercury::num_virtual];

    // curr_config_[mercury_joint::rightHip] = 
    //    data->joint_jpos[mercury_joint::rightHip - mercury::num_virtual];
    // curr_config_[mercury_joint::leftHip] = 
    //    data->joint_jpos[mercury_joint::leftHip - mercury::num_virtual];

    std::vector<double> imu_acc(3);
    std::vector<double> imu_ang_vel(3);
    std::vector<double> imu_inc(3);

    // Try derivative filter on joint position:
    for (int i(0); i<mercury::num_act_joint; ++i){
        filter_jpos_vel_[i]->input(data->joint_jpos[i]);       
        sp_->filtered_jvel_[i] = filter_jpos_vel_[i]->output(); 
    }        

    // Try low pass filter on ang velocity data
    for (int i(0); i < 3; ++i){
        filter_ang_vel_[i]->input(data->imu_ang_vel[i]);       
        sp_->filtered_ang_vel_[i] = filter_ang_vel_[i]->output(); 
    }        


    for(int i(0); i<3; ++i){
        imu_inc[i] = data->imu_inc[i];
        imu_acc[i]  = data->imu_acc[i];
        imu_ang_vel[i] = data->imu_ang_vel[i];
        //imu_ang_vel[i] = filter_ang_vel_[i]->output();
    }
    ori_est_->setSensorData(imu_acc, imu_inc, imu_ang_vel);
    ori_est_->getEstimatedState(sp_->body_ori_, sp_->body_ang_vel_);

    bias_vel_est_->setSensorData(imu_acc, sp_->body_ori_);
    bias_vel_est_->getEstimatedCoMState(sp_->com_state_imu_);


    // Use filtered imu angular velocity data for the EKF
    std::vector<double> filtered_imu_ang_vel(3);
    for(int i(0); i<3; ++i){
        filtered_imu_ang_vel[i] = filter_ang_vel_[i]->output();
    }

    // EKF set sensor data
    //ekf_est_->setSensorData(imu_acc, imu_inc, filtered_imu_ang_vel, 
    //data->lfoot_contact, 
    //data->rfoot_contact,
    //curr_config_.segment(mercury::num_virtual, mercury::num_act_joint),
    //curr_qdot_.segment(mercury::num_virtual, mercury::num_act_joint));

    static bool visit_once(false);
    if ((sp_->phase_copy_ == 2) && (!visit_once)){
        //ekf_est_->resetFilter();
        vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        mocap_vel_est_->Initialization(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
        body_foot_est_->Initialization(sp_->body_ori_);
        visit_once = true;
    }

    dynacore::Quaternion ekf_quaternion_est;
    //ekf_est_->getEstimatedState(sp_->ekf_body_pos_, sp_->ekf_body_vel_, ekf_quaternion_est); // EKF    

    if(base_cond_ == base_condition::floating){
        curr_config_[3] = sp_->body_ori_.x();
        curr_config_[4] = sp_->body_ori_.y();
        curr_config_[5] = sp_->body_ori_.z();
        curr_config_[mercury::num_qdot] = sp_->body_ori_.w();

        for(int i(0); i<3; ++i)
            curr_qdot_[i+3] = sp_->body_ang_vel_[i];


        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        // Foot position based offset
        dynacore::Vect3 foot_pos, foot_vel;
        robot_sys_->getPos(sp_->stance_foot_, foot_pos);
        robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);

        curr_config_[0] = -foot_pos[0];
        curr_config_[1] = -foot_pos[1];
        curr_config_[2] = -foot_pos[2];
        curr_qdot_[0] = -foot_vel[0];
        curr_qdot_[1] = -foot_vel[1];
        curr_qdot_[2] = -foot_vel[2];

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

        /// Jpos based model update  ////////////////////////////////
        if(b_jpos_model_update_){
            jjpos_config_[3] = sp_->body_ori_.x();
            jjpos_config_[4] = sp_->body_ori_.y();
            jjpos_config_[5] = sp_->body_ori_.z();
            jjpos_config_[mercury::num_qdot] = sp_->body_ori_.w();
            for(int i(0); i<3; ++i)
                jjvel_qdot_[i+3] = sp_->body_ang_vel_[i];

            sp_->jjpos_robot_sys_->UpdateSystem(jjpos_config_, jjvel_qdot_);

            sp_->jjpos_robot_sys_->getPos(sp_->stance_foot_, foot_pos);
            sp_->jjpos_robot_sys_->getLinearVel(sp_->stance_foot_, foot_vel);
            jjpos_config_[0] = -foot_pos[0];
            jjpos_config_[1] = -foot_pos[1];
            jjpos_config_[2] = -foot_pos[2];
            jjvel_qdot_[0] = -foot_vel[0];
            jjvel_qdot_[1] = -foot_vel[1];
            jjvel_qdot_[2] = -foot_vel[2];

            sp_->jjpos_robot_sys_->UpdateSystem(jjpos_config_, jjvel_qdot_);
            sp_->jjpos_config_ = jjpos_config_;
            sp_->jjvel_qdot_ = jjvel_qdot_;
        }
        /// END of Jpos based model update  /////////////////////////
    } else if (base_cond_ == base_condition::fixed){
        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);

    } else if (base_cond_ == base_condition::lying){
        // pitch rotation (PI/2)
        curr_config_[4] = sin(M_PI/2.0/2.0);
        curr_config_[mercury::num_qdot] = cos(M_PI/2.0/2.0);

        robot_sys_->UpdateSystem(curr_config_, curr_qdot_);
    } else {
        printf("[Error] Incorrect base condition setup\n");
        exit(0);
    }
    sp_->Q_ = curr_config_;
    sp_->Qdot_ = curr_qdot_;

    // Warning: Save Sensor Data in StateProvider
    // LED data save  ////////////////////
    dynacore::Vect3 pos;
    int led_idx = mercury_link::LED_BODY_0;
    for(int i(0); i<NUM_MARKERS; ++i){
        robot_sys_->getPos(mercury_link::LED_BODY_0 + i, pos);
        for (int j(0); j<3; ++j)  sp_->led_kin_data_[3*i + j] = pos[j];
    }
    //////////////////////////////////////
    robot_sys_->getCoMPosition(sp_->CoM_pos_);
    robot_sys_->getCoMVelocity(sp_->CoM_vel_);

    // CoM velocity data smoothing filter
    vel_est_->Update(sp_->CoM_vel_[0], sp_->CoM_vel_[1]);
    vel_est_->Output(sp_->est_CoM_vel_[0], sp_->est_CoM_vel_[1]);

    // Mocap based body velocity estimator
    dynacore::Vect3 mocap_body_vel;
    body_foot_est_->Update();
    body_foot_est_->getMoCapBodyVel(mocap_body_vel);
    body_foot_est_->getMoCapBodyPos(sp_->body_ori_, sp_->est_mocap_body_pos_);
    mocap_vel_est_->Update(mocap_body_vel[0], mocap_body_vel[1]);
    mocap_vel_est_->Output(
            sp_->est_mocap_body_vel_[0], 
            sp_->est_mocap_body_vel_[1]);

    // Right Contact 
    if(data->rfoot_contact) sp_->b_rfoot_contact_ = 1;
    else sp_->b_rfoot_contact_ = 0;
    // Left Contact 
    if(data->lfoot_contact) sp_->b_lfoot_contact_ = 1;
    else sp_->b_lfoot_contact_ = 0;

    for(int i(0); i<3; ++i){
        sp_->imu_acc_inc_[i] = data->imu_inc[i];
        sp_->imu_acc_[i] = data->imu_acc[i];
        sp_->imu_ang_vel_[i] = data->imu_ang_vel[i];
    }
    // Foot pos
    robot_sys_->getPos(mercury_link::rightFoot, sp_->Rfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::rightFoot, sp_->Rfoot_vel_);
    robot_sys_->getPos(mercury_link::leftFoot, sp_->Lfoot_pos_);
    robot_sys_->getLinearVel(mercury_link::leftFoot, sp_->Lfoot_vel_);

    for (int i(0); i<3; ++i){
        sp_->jjpos_body_pos_[i] = jjpos_config_[i];
        sp_->jjvel_body_vel_[i] = jjvel_qdot_[i];        
    }
    sp_->SaveCurrentData();
}
