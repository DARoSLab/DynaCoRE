#ifndef HUME_DYN_MODEL
#define HUME_DYN_MODEL

#include "utils/Sejong_Thread.h"
#include "utils/wrap_eigen.hpp"
#include <rbdl/rbdl.h>

class StateProvider;

using namespace sejong;

class Hume_Dyn_Model: public Sejong_Thread{
public:

    Hume_Dyn_Model(RigidBodyDynamics::Model* model);
    virtual ~Hume_Dyn_Model(void);

    virtual void run(void);

    bool getInverseMassInertia(Matrix & ainv);
    bool getGravity(Vector &  grav);
    bool getCoriolis(Vector & coriolis);

    // void UpdateDynamics();
    void UpdateDynamics(const sejong::Vector & q, const sejong::Vector & qdot);
    
protected:
    Matrix Ainv_;
    Vector grav_;
    Vector coriolis_;
    double sigmaThreshold_;

    RigidBodyDynamics::Model* model_;
};

#endif
