#ifndef ORIENTATION_ESTIMATOR_ATLAS
#define ORIENTATION_ESTIMATOR_ATLAS

#include <Utils/wrap_eigen.hpp>

class OriEstimator{
    public:
        OriEstimator(){
            global_ori_.w() = 1.;
            global_ori_.x() = 0.;
            global_ori_.y() = 0.;
            global_ori_.z() = 0.;

            global_ang_vel_.setZero();
        }


        virtual ~OriEstimator(){}

        virtual void setSensorData(const std::vector<double> & acc,
                const std::vector<double> & ang_vel) = 0;

        virtual void EstimatorInitialization(
                const dynacore::Quaternion & ini_quat,
                const std::vector<double> & acc,
                const std::vector<double> & ang_vel) = 0;


        void getEstimatedState(dynacore::Quaternion & ori,
                dynacore::Vect3 & global_ang_vel){
            ori = global_ori_;
            global_ang_vel = global_ang_vel_;
        }

    protected:
        dynacore::Quaternion global_ori_;
        dynacore::Vect3 global_ang_vel_;
};

#endif
