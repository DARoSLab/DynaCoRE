#include "Hume_Kin_Model.h"

#include "utils/pseudo_inverse.hpp"
#include "utils/utilities.h"

#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>

using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

Hume_Kin_Model::Hume_Kin_Model( RigidBodyDynamics::Model* model){
    model_ = model;
}

Hume_Kin_Model::~Hume_Kin_Model(){
}
void Hume_Kin_Model::getCentroidJacobian(sejong::Matrix & J){
    double mass;
    Vector3d zero_vector;
    zero_vector.setZero();

    Vector3d com_pos;
    Vector3d cm;
    Vector3d link_pos;
    Vector3d p_g;

    sejong::Vector q;
    getCoMPos(q, com_pos, false);
    
    Matrix Xg_inv = Matrix::Zero(6, 6);
    Matrix Ig = Matrix::Zero(6, 6);
    Matrix Ag = Matrix::Zero(6, model_->qdot_size);

    Matrix I = Matrix::Zero(6, 6);
    Matrix Jsp = Matrix::Zero(6, model_->qdot_size);
    
    int start_idx = _find_body_idx(HIP);
    Matrix3d p;
    Matrix3d cmm;
    Matrix3d R;
    for (int i(start_idx); i<model_->mBodies.size(); ++i){
        R = CalcBodyWorldOrientation(*model_, q, i, false);
        
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i, zero_vector, false);

        Jsp.setZero();
        CalcBodySpatialJacobian( *model_, q, i, Jsp, false);
        
        mass = model_->mBodies[i].mMass;
        I.setZero();
        cm = model_->mBodies[i].mCenterOfMass;
        cmm <<
            0.0, -cm[2], cm[1],
            cm[2], 0.0, -cm[0],
            -cm[1], cm[0], 0.0;
        I.setZero();
        I.block(0, 0, 3, 3) = model_->mBodies[i].mInertia + mass * cmm * cmm.transpose();
        I.block(0,3, 3,3) = mass * cmm;
        I.block(3,0, 3,3) = -mass * cmm;
        I.block(3, 3, 3, 3) = mass * Matrix::Identity(3,3);

        p_g = R * (com_pos - link_pos);
        p << 0.0, -p_g[2], p_g[1],
            p_g[2], 0.0, -p_g[0],
            -p_g[1], p_g[0], 0.0;
        
        Xg_inv.block(0,0, 3,3) = R;
        Xg_inv.block(3,3, 3,3) = R;
        Xg_inv.block(3,0, 3,3) = p * R;
        Ig = Ig + Xg_inv.transpose() * I * Xg_inv;
        Ag = Ag + Xg_inv.transpose() * I * Jsp;
    }
    
    J = Ig.inverse() * Ag;
    sejong::pretty_print(Ig, std::cout, "Ig", "");
    sejong::pretty_print(Ag, std::cout, "Ag", "");
}

void Hume_Kin_Model::getCoMJacobian(const sejong::Vector & Q, sejong::Matrix & Jcom) const {
    Vector3d zero_vector = Vector3d::Zero();
    
    Jcom = Matrix::Zero(3, model_->qdot_size);
    MatrixNd J(3, model_->qdot_size);
    
    double mass;
    double tot_mass(0.0);
    int start_idx = _find_body_idx(HIP);
    
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        // CoM Jacobian Update
        J.setZero();
        CalcPointJacobian(*model_, Q, i, model_->mBodies[i].mCenterOfMass, J, false);
        Jcom +=  mass * J;
                
        tot_mass += mass;
    }
    Jcom /= tot_mass;
}

void Hume_Kin_Model::getCoMPos(const sejong::Vector & q, sejong::Vect3 & CoM_pos, bool update)const {
    Vector3d zero_vector = Vector3d::Zero();
        
    CoM_pos.setZero();
    Vector3d link_pos;

    int start_idx = _find_body_idx(HIP);
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;

        // CoM position Update
        link_pos = CalcBodyToBaseCoordinates ( *model_, q, i,  model_->mBodies[i].mCenterOfMass, update);
        CoM_pos += mass * link_pos;
        tot_mass += mass;
    }
    CoM_pos /= tot_mass;
}

void Hume_Kin_Model::getCoMVel(const sejong::Vector & q, const sejong::Vector & qdot, sejong::Vect3 & CoM_vel) const {
    
    int start_idx = _find_body_idx(HIP);
    CoM_vel = sejong::Vector::Zero(3);
    Vector3d link_vel;

    Vector3d zero_vector = Vector3d::Zero();
    double mass;
    double tot_mass(0.0);
    for (int i(start_idx); i< model_->mBodies.size() ; ++i){
        mass = model_->mBodies[i].mMass;
        
        // CoM velocity Update
        link_vel = CalcPointVelocity ( *model_, q, qdot, i, model_->mBodies[i].mCenterOfMass, false);
        CoM_vel += mass * link_vel;
        tot_mass += mass;
    }
    CoM_vel /= tot_mass;
}

void Hume_Kin_Model::getPosition(const Vector & q, int link_id, Vect3 & pos){
    Vector3d zero;
    // zero << 0.0, 0.0, 0.0;
    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    pos = CalcBodyToBaseCoordinates(*model_, q, _find_body_idx(link_id), zero, false);
    // pos = CalcBaseToBodyCoordinates(*model_, q, _find_body_idx(link_id), zero, false);

}

void Hume_Kin_Model::getOrientation(const Vector & q, int link_id, sejong::Quaternion & ori){
    ori = CalcBodyWorldOrientation( *model_, q, _find_body_idx(link_id), false);
}
void Hume_Kin_Model::getVelocity(const Vector & q, const Vector &qdot,
                                 int link_id, Vect3 & vel){
    Vector3d zero;
    // zero << 0.0, 0.0, 0.0;
    int bodyid = _find_body_idx(link_id);
    if(bodyid >=model_->fixed_body_discriminator){
        zero = model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass;
    }
    else{
        zero =  model_->mBodies[bodyid].mCenterOfMass;
    }

    vel = CalcPointVelocity ( *model_, q, qdot, _find_body_idx(link_id), zero, false);

}
void Hume_Kin_Model::getAngVel(const Vector & q, const Vector & qdot,
                               int link_id, Vect3 & ang_vel){
    MatrixNd J(6, model_->qdot_size);
    J.setZero();
    Vector spatial_velocity;
    CalcBodySpatialJacobian( *model_, q, _find_body_idx(link_id), J, false);
    spatial_velocity = J * qdot;
    ang_vel = spatial_velocity.head(3);
}

// Angular velocity: from local (body) coordinate
// Linear velocity: from global coordinate
void Hume_Kin_Model::getJacobian(const Vector & q, int link_id, Matrix &J){
    J = Matrix::Zero(6, model_->qdot_size);
    Matrix J_lin = Matrix::Zero(3, model_->qdot_size);
    Matrix J_w = Matrix::Zero(6, model_->qdot_size);
    
    unsigned int bodyid = _find_body_idx(link_id);

    Vector3d zero_vector = Vector3d::Zero();

    if(bodyid >=model_->fixed_body_discriminator){
        CalcPointJacobian(*model_, q, bodyid,
                          model_->mFixedBodies[bodyid - model_->fixed_body_discriminator].mCenterOfMass,
                          J_lin, false);
    }
    else{
        CalcPointJacobian(*model_, q, bodyid,
                          model_->mBodies[bodyid].mCenterOfMass,
                          J_lin, false);

    }
    // Matrix J_foot = Matrix::Zero(3, model_->qdot_size);
    // CalcPointJacobian(*model_, q, _find_body_idx(RCalf),
    //                   zero_vector,
    //                   J_foot, false);
    
    // sejong::pretty_print(J_foot, std::cout, "JShank_zerovector", "");
    // J_foot.setZero();
    // CalcPointJacobian(*model_, q, _find_body_idx(RCalf),
    //                   model_->mBodies[_find_body_idx(RCalf)].mCenterOfMass,
    //                   J_foot, false);

    // sejong::pretty_print(J_foot, std::cout, "JShank_CoM", "");
    
    CalcBodySpatialJacobian( *model_, q, _find_body_idx(link_id), J_w, false);

    J.block(0,0, 3, model_->qdot_size) = J_w.block(0,0, 3, model_->qdot_size);
    J.block(3,0, 3, model_->qdot_size) = J_lin;
}

unsigned int Hume_Kin_Model::_find_body_idx(int id) const {
    // printf("find %d\n", id);
    switch (id){
    case RFOOT:
        return model_->GetBodyId("rfoot");
    case LFOOT:
        return model_->GetBodyId("lfoot");
    case IMU:
        return model_->GetBodyId("imu");
    case LED_BODY_0: 
    case LED_BODY_1: 
    case LED_BODY_2: 
    case LED_BODY_3: 
        return model_->GetBodyId("body_led3") + (id - LED_BODY_0);
    case LED_LEG1_0:
        return model_->GetBodyId("led0");
    case LED_LEG1_1:
        return model_->GetBodyId("led2");
    case LED_LEG1_2:
        return model_->GetBodyId("led4");
    case LED_LEG2_0:
        return model_->GetBodyId("led7");
    case LED_LEG2_1:
        return model_->GetBodyId("led9");
    }
    
    return (unsigned int)(id + 2);
}
