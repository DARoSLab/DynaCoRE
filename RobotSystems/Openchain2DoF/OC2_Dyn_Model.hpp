#ifndef OPENCHAIN_2DOF__DYN_MODEL
#define OPENCHAIN_2DOF__DYN_MODEL

#include <Utils/wrap_eigen.hpp>
#include <rbdl/rbdl.h>

using namespace sejong;

class OC2_Dyn_Model{
public:

    OC2_Dyn_Model(RigidBodyDynamics::Model* model);
    ~OC2_Dyn_Model(void);

    bool getMassInertia(Matrix & a);
    bool getInverseMassInertia(Matrix & ainv);
    bool getGravity(Vector &  grav);
    bool getCoriolis(Vector & coriolis);

    void UpdateDynamics(const sejong::Vector & q, const sejong::Vector & qdot);

protected:
    Matrix A_;
    Matrix Ainv_;
    Vector grav_;
    Vector coriolis_;

    RigidBodyDynamics::Model* model_;
};

#endif
