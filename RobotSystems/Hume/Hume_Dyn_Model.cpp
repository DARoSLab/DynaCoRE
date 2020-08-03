#include "Hume_Dyn_Model.h"

// #include "utils/pseudo_inverse.hpp"
#include "utils/utilities.h"

#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>
// #include <ControlSystem/Hume_Controller/StateProvider.h>
#include "rbdl/urdfreader.h"

// using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

Hume_Dyn_Model::Hume_Dyn_Model(RigidBodyDynamics::Model* model): Sejong_Thread(){
    // state_provider_ = StateProvider::GetStateProvider();
    model_ = model;
    sigmaThreshold_ = 0.00000001;
}

Hume_Dyn_Model::~Hume_Dyn_Model(){
}

void Hume_Dyn_Model::run()
{    
    // printf("[Hume Dynamic Model] Start, TID: %d \n", (int)syscall(SYS_gettid));
    // while (true)
    // {
    //     UpdateDynamics();
    //     usleep(3000);
    // }
}
bool Hume_Dyn_Model::getInverseMassInertia(Matrix & ainv){
    ainv = Ainv_;
    return true;
}
bool Hume_Dyn_Model::getGravity(Vector &  grav){
    grav = grav_;
    return true;
}
bool Hume_Dyn_Model::getCoriolis(Vector & coriolis){
    coriolis = coriolis_;
    return true;
}

// void Hume_Dyn_Model::UpdateDynamics(){
//     if( state_provider_->Q_.rows() < model_->q_size) { return ;}
//     // Mass Matrix
//     Matrix A = Matrix::Zero(model_->qdot_size, model_->qdot_size);
//     CompositeRigidBodyAlgorithm(*model_, state_provider_->Q_, A, false);

//     pseudoInverse(A, sigmaThreshold_, Ainv_, 0); 

//     Vector ZeroQdot = Vector::Zero(model_->qdot_size);
//     // Gravity
//     Vector grav_tmp = sejong::Vector::Zero(model_->qdot_size);
//     InverseDynamics(*model_, state_provider_->Q_, ZeroQdot, ZeroQdot, grav_tmp);
//     grav_ = grav_tmp;
//     // Coriolis
//     Vector coriolis_tmp = sejong::Vector::Zero(model_->qdot_size);
//     InverseDynamics(*model_, state_provider_->Q_, state_provider_->Qdot_, ZeroQdot, coriolis_tmp);

//     coriolis_ = coriolis_tmp - grav_;
// }

void Hume_Dyn_Model::UpdateDynamics(const sejong::Vector & q, const sejong::Vector & qdot){
    // Mass Matrix
    Matrix A = Matrix::Zero(model_->qdot_size, model_->qdot_size);
    CompositeRigidBodyAlgorithm(*model_, q, A, false);

    Ainv_ = A.inverse();
    // pseudoInverse(A, sigmaThreshold_, Ainv_, 0); 

    Vector ZeroQdot = Vector::Zero(model_->qdot_size);
    // Gravity
    Vector grav_tmp = sejong::Vector::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, ZeroQdot, ZeroQdot, grav_tmp);
    grav_ = grav_tmp;
    // Coriolis
    Vector coriolis_tmp = sejong::Vector::Zero(model_->qdot_size);
    InverseDynamics(*model_, q, qdot, ZeroQdot, coriolis_tmp);

    coriolis_ = coriolis_tmp - grav_;
} 
