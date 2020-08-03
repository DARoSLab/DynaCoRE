#ifndef STATE_ESTIMATOR_TELLO
#define STATE_ESTIMATOR_TELLO

#include <Configuration.h>
#include <Utils/wrap_eigen.hpp>


class TELLO_StateProvider;
class RobotSystem;
class BasicAccumulation;
class TELLO_SensorData;

class TELLO_StateEstimator{
    public:
        TELLO_StateEstimator(RobotSystem* robot);
        ~TELLO_StateEstimator();

        void Initialization(TELLO_SensorData* );
        void Update(TELLO_SensorData* );

    protected:
        double initial_height_;
        int fixed_foot_;
        dynacore::Vect3 foot_pos_;
        TELLO_StateProvider* sp_;
        RobotSystem* robot_sys_;

        dynacore::Vector curr_config_;
        dynacore::Vector curr_qdot_;

        BasicAccumulation* ori_est_;
};

#endif
