#ifndef STATE_ESTIMATOR_Humanoid
#define STATE_ESTIMATOR_Humanoid

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class Humanoid_StateProvider;
class RobotSystem;
class BasicAccumulation;
class Humanoid_SensorData;

class Humanoid_StateEstimator{
    public:
        Humanoid_StateEstimator(RobotSystem* robot);
        ~Humanoid_StateEstimator();

        void Initialization(Humanoid_SensorData* );
        void Update(Humanoid_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        Humanoid_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
};

#endif
