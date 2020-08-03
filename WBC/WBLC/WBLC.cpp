#include "WBLC.hpp"
#include <Utils/utilities.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

WBLC::WBLC(const std::vector<bool> & act_list,
        const dynacore::Matrix * Jci): 
    WBC(act_list, Jci){
    Sf_ = dynacore::Matrix::Zero(6, num_qdot_);
    Sf_.block(0,0, 6, 6).setIdentity();

    act_list_.clear();
    for(int i(0); i<num_qdot_; ++i){
        if(act_list[i]) act_list_.push_back(i);
    }
    qddot_ = dynacore::Vector::Zero(num_qdot_);
   //dynacore::pretty_print(Sv_, std::cout, "Sv");
}

void WBLC::UpdateSetting(const dynacore::Matrix & A,
        const dynacore::Matrix & Ainv,
        const dynacore::Vector & cori,
        const dynacore::Vector & grav,
        void* extra_setting){

    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;

    //dynacore::pretty_print(grav_, std::cout, "grav");
    //dynacore::pretty_print(cori_, std::cout, "cori");
    //dynacore::pretty_print(A_, std::cout, "A");
}

void WBLC::MakeWBLC_Torque(
        const dynacore::Vector & des_jacc_cmd,
        const std::vector<ContactSpec*> & contact_list,
        dynacore::Vector & cmd,
        void* extra_input){
    
    if(!b_updatesetting_) { printf("[Wanning] WBLC setting is not done\n"); }
    if(extra_input) data_ = static_cast<WBLC_ExtraData*>(extra_input);

    // Internal Constraint Check
    Nci_ = dynacore::Matrix::Identity(num_qdot_, num_qdot_);

    if(b_internal_constraint_) {
        dynacore::Matrix JciBar;
        _WeightedInverse(Jci_, Ainv_, JciBar);
        Nci_ -= JciBar * Jci_;
    }
    for(int i(0); i<num_act_joint_; ++i){
        qddot_[act_list_[i]] = des_jacc_cmd[i];
    }

    // Contact Jacobian & Uf & Fr_ieq
    _BuildContactMtxVect(contact_list);

    // Dimension Setting
    dim_opt_ = num_qdot_ + 2 * dim_rf_; // (delta_qddot, Fr, xddot_c)
    dim_eq_cstr_ = num_passive_ + dim_rf_;
    dim_ieq_cstr_ = 2*num_act_joint_ + Uf_.rows();

    _Build_Equality_Constraint();
    _Build_Inequality_Constraint();
    _OptimizationPreparation(Aeq_, beq_, Cieq_, dieq_);

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    _GetSolution(cmd);
    //std::cout << "f: " << f << std::endl;
    //std::cout << "x: " << z << std::endl;
    //std::cout << "cmd: "<<cmd<<std::endl;

    // if(f > 1.e5){
    //   std::cout << "f: " << f << std::endl;
    //   std::cout << "x: " << z << std::endl;
    //   std::cout << "cmd: "<<cmd<<std::endl;

    //   printf("G:\n");
    //   std::cout<<G<<std::endl;
    //   printf("g0:\n");
    //   std::cout<<g0<<std::endl;

    //   printf("CE:\n");
    //   std::cout<<CE<<std::endl;
    //   printf("ce0:\n");
    //   std::cout<<ce0<<std::endl;

    //   printf("CI:\n");
    //   std::cout<<CI<<std::endl;
    //   printf("ci0:\n");
    //   std::cout<<ci0<<std::endl;
    // }
}

void WBLC::_Build_Inequality_Constraint(){
    Cieq_ = dynacore::Matrix::Zero(dim_ieq_cstr_, dim_opt_);
    dieq_ = dynacore::Vector::Zero(dim_ieq_cstr_);
    int row_idx(0);

    Cieq_.block(row_idx, num_qdot_, Uf_.rows(), dim_rf_) = Uf_;
    dieq_.head(Uf_.rows()) = Fr_ieq_;
    row_idx += Uf_.rows();

    Cieq_.block(row_idx, 0, num_act_joint_, num_qdot_) = Sa_ * A_;
    Cieq_.block(row_idx, num_qdot_, num_act_joint_, dim_rf_) = -Sa_ * Jc_.transpose();
    dieq_.segment(row_idx, num_act_joint_) 
        = data_->tau_min_ - Sa_ * (cori_ + grav_ + A_ * qddot_);
    row_idx += num_act_joint_;

    Cieq_.block(row_idx, 0, num_act_joint_, num_qdot_) = -Sa_ * A_;
    Cieq_.block(row_idx, num_qdot_, num_act_joint_, dim_rf_) = Sa_ * Jc_.transpose();
    dieq_.segment(row_idx, num_act_joint_) 
        = -data_->tau_max_ + Sa_ * (cori_ + grav_ + A_ * qddot_);

    //dynacore::pretty_print(Cieq_, std::cout, "C ieq");
    //dynacore::pretty_print(dieq_, std::cout, "d ieq");
}

void WBLC::_Build_Equality_Constraint(){
    Aeq_ = dynacore::Matrix::Zero(dim_eq_cstr_, dim_opt_);
    beq_ = dynacore::Vector::Zero(dim_eq_cstr_);

    // passive joint
    Aeq_.block(0, 0, num_passive_, num_qdot_) = Sv_ * A_;
    Aeq_.block(0, num_qdot_, num_passive_, dim_rf_) = -Sv_ * Jc_.transpose();
    beq_.head(num_passive_) = -Sv_ * (A_ * qddot_ + cori_ + grav_);
    
    // xddot
    Aeq_.block(num_passive_, 0, dim_rf_, num_qdot_) = Jc_;
    Aeq_.bottomRightCorner(dim_rf_, dim_rf_) = 
        -dynacore::Matrix::Identity(dim_rf_, dim_rf_);
    beq_.tail(dim_rf_) = -Jc_ * qddot_ - JcDotQdot_;

    //dynacore::pretty_print(Aeq_, std::cout, "Aeq");
    //dynacore::pretty_print(beq_, std::cout, "beq");
}

void WBLC::_BuildContactMtxVect(const std::vector<ContactSpec*> & contact_list){
    WBDC_ContactSpec* contact = ((WBDC_ContactSpec*)contact_list[0]);
    contact->getContactJacobian(Jc_);
    contact->getJcDotQdot(JcDotQdot_);
    contact->getRFConstraintMtx(Uf_);
    contact->getRFConstraintVec(Fr_ieq_);

    dynacore::Matrix Jc_i, Uf_i;
    dynacore::Vector Fr_ieq_i, JcDotQdot_i;

    dim_rf_ = Jc_.rows();
    int num_rows_Uf = Uf_.rows();
    for(int i(1); i<contact_list.size(); ++i){
        contact = ((WBDC_ContactSpec*)contact_list[i]);
        contact->getContactJacobian(Jc_i);
        contact->getJcDotQdot(JcDotQdot_i);
        contact->getRFConstraintMtx(Uf_i);
        contact->getRFConstraintVec(Fr_ieq_i);

        Jc_.conservativeResize(dim_rf_ + Jc_i.rows(), num_qdot_);
        Jc_.block(dim_rf_, 0, Jc_i.rows(), num_qdot_) = Jc_i;

        JcDotQdot_.conservativeResize(dim_rf_ + Jc_i.rows());
        JcDotQdot_.tail(Jc_i.rows()) = JcDotQdot_i;

        Uf_.conservativeResize(num_rows_Uf + Uf_i.rows(), dim_rf_ + Uf_i.cols());
        (Uf_.topRightCorner(num_rows_Uf, Uf_i.cols() ) ).setZero();
        (Uf_.bottomLeftCorner(Uf_i.rows(), dim_rf_) ).setZero();
        Uf_.block(num_rows_Uf, dim_rf_, Uf_i.rows(), Uf_i.cols()) = Uf_i;

        Fr_ieq_.conservativeResize(num_rows_Uf + Uf_i.rows());
        Fr_ieq_.tail(Uf_i.rows()) = Fr_ieq_i;

        dim_rf_ += Jc_i.rows();
        num_rows_Uf += Uf_i.rows();
    }
    //dynacore::pretty_print(Jc_, std::cout, "Jc"); 
    //dynacore::pretty_print(Uf_, std::cout, "Uf"); 
    //dynacore::pretty_print(JcDotQdot_, std::cout, "JcDotQdot"); 
    //dynacore::pretty_print(Fr_ieq_, std::cout, "Fr_ieq"); 
}
void WBLC::_OptimizationPreparation(
        const dynacore::Matrix & Aeq, 
        const dynacore::Vector & beq,
        const dynacore::Matrix & Cieq,
        const dynacore::Vector & dieq){
    G.resize(dim_opt_, dim_opt_); 
    g0.resize(dim_opt_);
    CE.resize(dim_opt_, dim_eq_cstr_);
    ce0.resize(dim_eq_cstr_);
    CI.resize(dim_opt_, dim_ieq_cstr_);
    ci0.resize(dim_ieq_cstr_);

    for(int i(0); i<dim_opt_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            G[i][j] = 0.;
        }
        g0[i] = 0.;
    }
    // Set Cost
    for (int i(0); i < num_qdot_; ++i){
        G[i][i] = data_->W_qddot_[i];
    }
    int idx_offset = num_qdot_;
    for (int i(0); i < dim_rf_; ++i){
        G[i + idx_offset][i + idx_offset] = data_->W_rf_[i];
    }
    idx_offset += dim_rf_;
    for (int i(0); i < dim_rf_; ++i){
        G[i + idx_offset][i + idx_offset] = data_->W_xddot_[i];
    }

    for(int i(0); i< dim_eq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CE[j][i] = Aeq(i,j);
        }
        ce0[i] = -beq[i];
    }

    for(int i(0); i< dim_ieq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CI[j][i] = Cieq(i,j);
        }
        ci0[i] = -dieq[i];
    }
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
}


void WBLC::MakeTorque(const std::vector<Task*> & task_list,
        const std::vector<ContactSpec*> & contact_list,
        dynacore::Vector & cmd,
        void* extra_input){
}

void WBLC::_GetSolution(dynacore::Vector & cmd){

    dynacore::Vector delta_qddot(num_qdot_);
    for(int i(0); i<num_qdot_; ++i) delta_qddot[i] = z[i];
    data_->Fr_ = dynacore::Vector(dim_rf_);
    for(int i(0); i<dim_rf_; ++i) data_->Fr_[i] = z[i + num_qdot_];

    dynacore::Vector tau = 
        A_ * (qddot_ + delta_qddot) + cori_ + grav_ - Jc_.transpose() * data_->Fr_;

    data_->qddot_ = qddot_ + delta_qddot;
    cmd = Sa_ * tau;

    //dynacore::pretty_print(qddot_, std::cout, "qddot_");
    //dynacore::pretty_print(delta_qddot, std::cout, "delta_qddot");
    //dynacore::pretty_print(data_->Fr_, std::cout, "Fr");
    //dynacore::pretty_print(tau, std::cout, "total tau");
    //dynacore::Vector x_check = Jc_ * (qddot_ + delta_qddot)  + JcDotQdot_;
    //dynacore::pretty_print(x_check, std::cout, "x check");
}


