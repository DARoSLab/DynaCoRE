#ifndef STATE_ESTIMATOR_SagitP3
#define STATE_ESTIMATOR_SagitP3

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class SagitP3_StateProvider;
class RobotSystem;
class OriEstimator;
class SagitP3_SensorData;

class SagitP3_StateEstimator{
    public:
        SagitP3_StateEstimator(RobotSystem* robot);
        ~SagitP3_StateEstimator();

        void Initialization(SagitP3_SensorData* );
        void Update(SagitP3_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        SagitP3_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        OriEstimator* ori_est_;

        void _RBDL_TEST();
};

#endif
