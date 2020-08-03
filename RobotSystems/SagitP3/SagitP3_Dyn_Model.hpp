#ifndef SagitP3_DYN_MODEL
#define SagitP3_DYN_MODEL

#include <rbdl/rbdl.h>
#include <Utils/wrap_eigen.hpp>

class SagitP3_Dyn_Model{
public:

    SagitP3_Dyn_Model(RigidBodyDynamics::Model* model);
    ~SagitP3_Dyn_Model(void);

    bool getMassInertia(dynacore::Matrix & a);
    bool getInverseMassInertia(dynacore::Matrix & ainv);
    bool getGravity(dynacore::Vector &  grav);
    bool getCoriolis(dynacore::Vector & coriolis);

    void UpdateDynamics(const dynacore::Vector & q, const dynacore::Vector & qdot);

protected:
    dynacore::Matrix A_;
    dynacore::Matrix Ainv_;
    dynacore::Vector grav_;
    dynacore::Vector coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
