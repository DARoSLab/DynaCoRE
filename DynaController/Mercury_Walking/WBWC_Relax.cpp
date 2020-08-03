#include "WBWC_Relax.hpp"
#include <Mercury_Controller/Mercury_StateProvider.hpp>

WBWC_Relax::WBWC_Relax(const RobotSystem* robot):
    robot_sys_(robot),
    torque_cmd_(mercury::num_act_joint),
    mu_(0.3),
    dim_rforce_(6),
    qddot_(mercury::num_qdot),
    left_z_min_(0.),
    left_z_max_(250.),
    right_z_min_(0.),
    right_z_max_(250.),
    Kp_(mercury::num_act_joint),
    Kd_(mercury::num_act_joint)
{
    Kp_.setZero();
    Kd_.setZero();
    W_rf_ = dynacore::Vector::Constant(dim_rforce_, 1.);
    W_foot_ = dynacore::Vector::Constant(dim_rforce_, 1.);
    W_virtual_ = dynacore::Vector::Constant(mercury::num_virtual, 1.);
    W_joint_ = dynacore::Vector::Constant(mercury::num_act_joint, 10000.);

    Jc_ = dynacore::Matrix::Zero(dim_rforce_, mercury::num_qdot);
    Fr_ = dynacore::Vector::Zero(dim_rforce_);

    dim_opt_ = mercury::num_virtual + dim_rforce_ + dim_rforce_ + 
        mercury::num_act_joint;

    sp_ = Mercury_StateProvider::getStateProvider();
    qddot_.setZero();
}

void WBWC_Relax::computeTorque(
        const dynacore::Vector & jpos_des,
        const dynacore::Vector & jvel_des,
        const dynacore::Vector & jacc_des, 
        dynacore::Vector & torque_cmd){
    //printf("wb\n");
    //dynacore::pretty_print(jacc_des, std::cout, "jacc_des");
    //dynacore::pretty_print(jpos_des, std::cout, "jpos_des");
    //dynacore::pretty_print(jvel_des, std::cout, "jvel_des");
    qddot_.setZero();
    for(int i(0); i<mercury::num_act_joint; ++i){
        qddot_[i + mercury::num_virtual] = jacc_des[i] + 
            Kp_[i] * (jpos_des[i] - sp_->Q_[mercury::num_virtual + i]) + 
            Kd_[i] * (jvel_des[i] - sp_->Qdot_[mercury::num_virtual + i]);
    }
    _SetWeightMatrix();
    _SetEqualityConstraint();
    _SetInEqualityConstraint();

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    _GetSolution(torque_cmd);

    // std::cout << "f: " << f << std::endl;
    // std::cout << "x: " << z << std::endl;
    // std::cout << "cmd: "<<torque_cmd<<std::endl;

    //printf("G:\n");
    //std::cout<<G<<std::endl;
    //printf("g0:\n");
    //std::cout<<g0<<std::endl;

    //printf("CE:\n");
    //std::cout<<CE<<std::endl;
    //printf("ce0:\n");
    //std::cout<<ce0<<std::endl;

    //printf("CI:\n");
    //std::cout<<CI<<std::endl;
    //printf("ci0:\n");
    //std::cout<<ci0<<std::endl;


    static int count(0);
    ++count;
    //if( count> 500 ) exit(0);
}

void WBWC_Relax::_SetEqualityConstraint(){
    // Ax = b
    // virtual 0 x6, foot acceleration
    int dim_eq = mercury::num_virtual + dim_rforce_;
    dynacore::Matrix A(dim_eq, dim_opt_); A.setZero();
    dynacore::Vector b(dim_eq); b.setZero();

    dynacore::Matrix Jc_tr = Jc_.transpose();

    // virtual joint
    A.block(0, 0, mercury::num_virtual, mercury::num_virtual) = 
        A_.block(0,0, mercury::num_virtual, mercury::num_virtual);
    A.block(0, mercury::num_virtual, mercury::num_virtual, dim_rforce_) = 
        - Jc_tr.block(0,0, mercury::num_virtual, dim_rforce_);
    A.block(0, mercury::num_virtual + 2*dim_rforce_, 
        mercury::num_virtual, mercury::num_act_joint) = 
        A_.block(0, mercury::num_virtual, mercury::num_virtual, mercury::num_act_joint);

    b.head(mercury::num_virtual) = 
        -b_.head(mercury::num_virtual) - g_.head(mercury::num_virtual);
    b.head(mercury::num_virtual) += 
        (-A_.block(0, mercury::num_virtual, mercury::num_virtual, mercury::num_act_joint)
        *qddot_.tail(mercury::num_act_joint) ) ;

    // foot acceleration
    A.block(mercury::num_virtual, 0, dim_rforce_, mercury::num_virtual) = 
        Jc_.block(0, 0, dim_rforce_, mercury::num_virtual);
    A.block(mercury::num_virtual, mercury::num_virtual + dim_rforce_, 
            dim_rforce_, dim_rforce_) = 
        -dynacore::Matrix::Identity(dim_rforce_, dim_rforce_);
    A.block(mercury::num_virtual, mercury::num_virtual + 2*dim_rforce_, 
        dim_rforce_, mercury::num_act_joint) = 
        Jc_.block(0, mercury::num_virtual, dim_rforce_, mercury::num_act_joint);

    b.tail(dim_rforce_) = -Jc_ * qddot_;

    CE.resize(dim_opt_, dim_eq);
    ce0.resize(dim_eq);

    for(int i(0); i< dim_eq; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CE[j][i] = A(i,j);
        }
        ce0[i] = -b[i];
    }
}

void WBWC_Relax::_SetInEqualityConstraint(){
    // C*x >= d
    int dim_ieq = 5*2 + 2;
    dynacore::Matrix C(dim_ieq, dim_opt_); C.setZero();
    dynacore::Vector d(dim_ieq); d.setZero();

    dynacore::Matrix U(5+1, 3); U.setZero();
    U(0,2) = 1.; // Z min
    U(1,2) = -1.; // - Z max
    U(2, 0) = 1.; U(2, 2) = mu_;
    U(3, 0) = -1.; U(3, 2) = mu_;
    U(4, 1) = 1.; U(4, 2) = mu_;
    U(5, 1) = -1.; U(5, 2) = mu_;

    d[0] = right_z_min_;
    d[1] = -right_z_max_;
    d[6] = left_z_min_;
    d[7] = -left_z_max_;
    // right contact
    C.block(0,mercury::num_virtual, 6, 3) = U;
    // left contact
    C.block(6, mercury::num_virtual + 3, 6, 3) = U;

    CI.resize(dim_opt_, dim_ieq);
    ci0.resize(dim_ieq);

   for(int i(0); i< dim_ieq; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CI[j][i] = C(i,j);
        }
        ci0[i] = -d[i];
    }
   //dynacore::pretty_print(C, std::cout, "inequality mtx");
   //dynacore::pretty_print(d, std::cout, "inequality vec");
}

void WBWC_Relax::_SetWeightMatrix(){
    G.resize(dim_opt_, dim_opt_);
    g0.resize(dim_opt_);

    for(int i(0); i<dim_opt_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }

    int offset(0);
    for(int i(0); i<mercury::num_virtual; ++i){
        G[i + offset][i + offset] = W_virtual_[i];
    }
    offset += mercury::num_virtual;
    for(int i(0); i < dim_rforce_; ++i){
        G[i + offset][i + offset] = W_rf_[i];
    }
    offset += dim_rforce_;
    for(int i(0); i < dim_rforce_; ++i){
        G[i + offset][i + offset] = W_foot_[i];
    }
    offset += dim_rforce_;
    for(int i(0); i < mercury::num_act_joint; ++i){
        G[i + offset][i + offset] = W_joint_[i];
    }
    //dynacore::pretty_print(W_virtual_, std::cout, "W virtual");
    //dynacore::pretty_print(W_rf_, std::cout, "W rf");
    //dynacore::pretty_print(W_foot_, std::cout, "W foot");
}

void WBWC_Relax::_GetSolution(dynacore::Vector & cmd){
    for(int i(0); i<mercury::num_virtual; ++i) qddot_[i] += z[i];
    for(int i(0); i<mercury::num_act_joint; ++i) {
        qddot_[i + mercury::num_virtual] += 
            z[i + mercury::num_virtual + dim_rforce_];
    }
    for(int i(0); i<dim_rforce_; ++i) Fr_[i] = z[i + mercury::num_virtual];
   
    dynacore::Vector full_torque = 
        A_ * qddot_ + b_ + g_ - Jc_.transpose() * Fr_;

    cmd = full_torque.tail(mercury::num_act_joint);

    //dynacore::pretty_print(full_torque, std::cout, "full torque");
}


