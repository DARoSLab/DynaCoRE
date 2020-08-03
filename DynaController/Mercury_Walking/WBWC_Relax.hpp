#ifndef WHOLE_BODY_WALKING_CONTROL_RELAXED
#define WHOLE_BODY_WALKING_CONTROL_RELAXED
// Only for Mercury

#include <Utils/wrap_eigen.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>
#include <Utils/utilities.hpp>
#include <Mercury/Mercury_Definition.h>
#include <Mercury/Mercury_Model.hpp>

class Mercury_StateProvider;

class WBWC_Relax{
    public:
        WBWC_Relax(const RobotSystem* robot);
        ~WBWC_Relax(){}

        void UpdateSetting(
                const dynacore::Matrix & A, 
                const dynacore::Vector & coriolis,
                const dynacore::Vector & grav){
            A_ = A;
            b_ = coriolis;
            g_ = grav;

            // TEST
            // b_.setZero();

            dynacore::Matrix Jtmp;
            robot_sys_->getFullJacobian(mercury_link::rightFoot, Jtmp);
            Jc_.block(0,0, 3, mercury::num_qdot) = 
                Jtmp.block(3, 0, 3, mercury::num_qdot);
            robot_sys_->getFullJacobian(mercury_link::leftFoot, Jtmp);
            Jc_.block(3,0, 3, mercury::num_qdot) = 
                Jtmp.block(3, 0, 3, mercury::num_qdot);
            qddot_.setZero();
        }

        void computeTorque(
                const dynacore::Vector & jpos_des,
                const dynacore::Vector & jvel_des,
                const dynacore::Vector & jacc_des,
                dynacore::Vector & torque_cmd);

        void updateWeight(
                const dynacore::Vector & W_virtual,
                const dynacore::Vector & W_rf,
                const dynacore::Vector & W_foot){
            W_rf_ = W_rf;
            W_virtual_ = W_virtual;
            W_foot_ = W_foot;
        }

        dynacore::Vector qddot_;
        dynacore::Vector Fr_;

        dynacore::Vector Kp_;
        dynacore::Vector Kd_;

        dynacore::Vector W_rf_; // right (x, y, z), left (x, y, z)
        dynacore::Vector W_virtual_;
        dynacore::Vector W_foot_;
        dynacore::Vector W_joint_;

        double left_z_min_;
        double left_z_max_;
        double right_z_min_;
        double right_z_max_;

 
    private:
        int dim_opt_;
        int dim_rforce_;

       double mu_;
        const RobotSystem* robot_sys_;
        const Mercury_StateProvider* sp_;

        dynacore::Vector torque_cmd_;

        dynacore::Matrix A_;
        dynacore::Matrix Jc_;
        dynacore::Vector b_;
        dynacore::Vector g_;

        void _SetInEqualityConstraint();
        void _SetEqualityConstraint();
        void _SetWeightMatrix();
        void _GetSolution(dynacore::Vector & cmd);

        // virtual (x, y, z, Rx, Ry, Rz)
        // full joint (6 - right abduction, hip, knee, left ...)
        // Reaction Forces (Right X, Right Y, Righ Z), (Left X, Left Y, Left Z)
        // Foot acceleration ''
        GolDIdnani::GVect<double> z;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;
        
        //min 0.5 * x G x + g0 x
        //s.t.
        //CE^T x + ce0 = 0
        //CI^T x + ci0 >= 0

        //The matrix and vectors dimensions are as follows:
        //G: n * n
        //g0: n

        //CE: n * p
        //ce0: p

        //CI: n * m
        //ci0: m

        //x: n


};

#endif
