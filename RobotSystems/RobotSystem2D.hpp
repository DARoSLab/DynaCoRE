#ifndef ROBOT_2D_MODEL
#define ROBOT_2D_MODEL

#include <Utils/wrap_eigen.hpp>
#include "RobotSystem.hpp"

// When users build 2D Model,
// they need to implement all functions in RobotSystem.hpp but
// in 2D return type (x, y, theta)

class RobotSystem2D: public RobotSystem{
    public:
        RobotSystem2D(): RobotSystem(){}
        virtual ~RobotSystem2D(void){}

        /* --------------------------------------*/
        //          Need to be implemented       //
        /* --------------------------------------*/

        //virtual bool getMassInertia(dynacore::Matrix & A) const = 0;
        //virtual bool getInverseMassInertia(dynacore::Matrix & Ainv) const = 0;
        //virtual bool getGravity(dynacore::Vector & grav) const = 0;
        //virtual bool getCoriolis(dynacore::Vector & coriolis) const = 0;
        //virtual void UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot) = 0;

        // (X, Y, theta)
        virtual void getPos(int link_id, dynacore::Vect3 & pos) const = 0;
        virtual void getVel(int link_id, dynacore::Vect3 & lin_vel) const = 0;

        virtual void getCentroidJacobian(dynacore::Matrix & Jcm) const = 0;
        virtual void getFullJacobian(int link_id, dynacore::Matrix & J) const = 0;
        virtual void getFullJacobianDot(int link_id, dynacore::Matrix & J) const = 0;

        // Don't need in 2D Model
        virtual void getCentroidVelocity(dynacore::Vector & centroid_vel) const {}
        virtual void getCoMJacobian(dynacore::Matrix & J) const {}

        virtual void getCentroidInertia(dynacore::Matrix & Icent) const {}
        virtual void getCoMPosition(dynacore::Vect3 & com_pos) const {}
        virtual void getCoMVelocity(dynacore::Vect3 & com_vel) const {}

        virtual void getOri(int link_id, dynacore::Quaternion & ori) const {}
        virtual void getLinearVel(int link_id, dynacore::Vect3 & lin_vel) const {}
        virtual void getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const {}
};

#endif
