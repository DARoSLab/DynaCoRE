#ifndef STATE_ESTIMATOR_Cheetah3
#define STATE_ESTIMATOR_Cheetah3

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class Cheetah3_StateProvider;
class RobotSystem;
class BasicAccumulation;
class Cheetah3_SensorData;

class Cheetah3_StateEstimator{
    public:
        Cheetah3_StateEstimator(RobotSystem* robot);
        ~Cheetah3_StateEstimator();

        void Initialization(Cheetah3_SensorData* );
        void Update(Cheetah3_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Cheetah3_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
};

#endif
