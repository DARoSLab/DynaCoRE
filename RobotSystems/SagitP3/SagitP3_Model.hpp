#ifndef SagitP3_MODEL
#define SagitP3_MODEL

#include <rbdl/rbdl.h>
#include <RobotSystem.hpp>

class SagitP3_Dyn_Model;
class SagitP3_Kin_Model;

class SagitP3_Model: public RobotSystem{
public:
    SagitP3_Model();
    virtual ~SagitP3_Model();

    virtual bool getMassInertia(dynacore::Matrix & A) const ;
    virtual bool getInverseMassInertia(dynacore::Matrix & Ainv) const; 
    virtual bool getGravity(dynacore::Vector & grav) const;
    virtual bool getCoriolis(dynacore::Vector & coriolis) const;

    virtual void getCentroidJacobian(dynacore::Matrix & Jcent) const;
    virtual void getCentroidInertia(dynacore::Matrix & Icent) const;
    virtual void getCoMPosition(dynacore::Vect3 & com_pos) const;
    virtual void getCoMVelocity(dynacore::Vect3 & com_vel) const;

    virtual void getPos(int link_id, dynacore::Vect3 & pos) const;
    virtual void getOri(int link_id, dynacore::Quaternion & ori) const;
    virtual void getLinearVel(int link_id, dynacore::Vect3 & lin_vel) const;
    virtual void getAngularVel(int link_id, dynacore::Vect3 & ang_vel) const;

    virtual void getCentroidVelocity(dynacore::Vector & centroid_vel) const;
    virtual void getCoMJacobian(dynacore::Matrix & J) const;

    virtual void getFullJacobian(int link_id, dynacore::Matrix & J) const;
    virtual void getFullJDotQdot(int link_id, dynacore::Vector & JDotQdot) const;

    virtual void UpdateSystem(const dynacore::Vector & q, const dynacore::Vector & qdot);

protected:
    SagitP3_Dyn_Model* dyn_model_;
    SagitP3_Kin_Model* kin_model_;

    RigidBodyDynamics::Model* model_;
};

#endif
