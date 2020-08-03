#ifndef STATE_ESTIMATOR_MERCURY
#define STATE_ESTIMATOR_MERCURY

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>
#include <Mercury_Controller/StateEstimator/BiasCompensatedBodyVelocityEstimator.hpp>


class Mercury_StateProvider;
class RobotSystem;
class filter;
class OriEstimator;
class EKF_PoseEstimator; // EKF estimator
class BodyFootPosEstimator;
class Mercury_SensorData;
class SimpleAverageEstimator;

class Mercury_StateEstimator{
    public:
        Mercury_StateEstimator(RobotSystem* robot);
        ~Mercury_StateEstimator();

        void Initialization(Mercury_SensorData* );
        void Update(Mercury_SensorData* );
        void setFloatingBase(int base_cond){ base_cond_ = base_cond; }
        void setJPosModelUpdate(bool b_enable){ b_jpos_model_update_ = b_enable; }

        bool b_using_jpos_;

    protected:
        bool b_jpos_model_update_;
        int base_cond_;
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Mercury_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        dynacore::Vector jjpos_config_;
        dynacore::Vector jjvel_qdot_;

        std::vector<filter*> filter_com_vel_;
        std::vector<filter*> filter_jpos_vel_;
        std::vector<filter*> filter_ang_vel_;


        BiasCompensatedBodyVelocityEstimator* bias_vel_est_; 
        OriEstimator* ori_est_;
        BodyFootPosEstimator* body_foot_est_;
        EKF_PoseEstimator* ekf_est_;  // EKF estimator
        SimpleAverageEstimator* vel_est_;
        SimpleAverageEstimator* mocap_vel_est_;

        std::vector<filter*> jvel_filter_;
};

#endif
