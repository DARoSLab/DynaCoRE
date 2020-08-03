#include "WBDC_Rotor.hpp"
#include <Utils/utilities.hpp>
#include <Eigen/LU>
#include <Eigen/SVD>

WBDC_Rotor::WBDC_Rotor(const std::vector<bool> & act_list,
        const dynacore::Matrix * Jci): WBC(act_list, Jci){
    Sf_ = dynacore::Matrix::Zero(6, num_qdot_);
    Sf_.block(0,0, 6, 6).setIdentity();
}

void WBDC_Rotor::UpdateSetting(const dynacore::Matrix & A,
        const dynacore::Matrix & Ainv,
        const dynacore::Vector & cori,
        const dynacore::Vector & grav,
        void* extra_setting){
    A_ = A;
    Ainv_ = Ainv;
    cori_ = cori;
    grav_ = grav;
    b_updatesetting_ = true;
}

bool WBDC_Rotor::_CheckNullSpace(const dynacore::Matrix & Npre){
    dynacore::Matrix M_check = Sf_ * A_ * Npre;
    dynacore::pretty_print(M_check,std::cout, "M check");
    // Eigen::JacobiSVD<dynacore::Matrix> svd(M_check, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::JacobiSVD<dynacore::Matrix> svd(Npre, Eigen::ComputeThinU | Eigen::ComputeThinV);

    //dynacore::pretty_print(svd.singularValues(), std::cout, "svd singular value");

    for(int i(0); i<svd.singularValues().rows(); ++i){
        if(svd.singularValues()[i] > 0.00001) { 
            printf("non singular!!\n"); 
            dynacore::pretty_print(svd.singularValues(), std::cout, "svd singular value");
            exit(0);
            return false;
        }else{
    //        printf("small enough singular value: %f\n", svd.singularValues()[i]);
        }
    }
    return true;
}

void WBDC_Rotor::MakeTorque(const std::vector<Task*> & task_list,
        const std::vector<ContactSpec*> & contact_list,
        dynacore::Vector & cmd,
        void* extra_data){
    _PrintDebug(1);    
    if(!b_updatesetting_) { printf("[Wanning] WBDC_Rotor setting is not done\n"); }

    if(extra_data) data_ = static_cast<WBDC_Rotor_ExtraData*>(extra_data);
    // Internal Constraint Check
    Nci_ = dynacore::Matrix::Identity(num_qdot_, num_qdot_);

    if(b_internal_constraint_) {
        dynacore::Matrix JciBar;
        _WeightedInverse(Jci_, Ainv_, JciBar);
        Nci_ -= JciBar * Jci_;
    }
    
    _PrintDebug(2);    
    // Contact Setting 
    _ContactBuilding(contact_list);
    dynacore::Matrix JcN = Jc_ * Nci_;
    dynacore::Matrix check = JcN - Jc_;

    _PrintDebug(3);    
    dynacore::Matrix JcN_Bar;
    _WeightedInverse(JcN, Ainv_, JcN_Bar);
    dynacore::Matrix Npre = dynacore::Matrix::Identity(num_qdot_, num_qdot_)
        - JcN_Bar * JcN;

    _PrintDebug(4);    
    // First Task Check
    Task* task = task_list[0];
    dynacore::Matrix Jt, JtPre, JtPreBar;
    dynacore::Vector JtDotQdot, xddot, qddot_pre;
    qddot_pre = JcN_Bar * ( - JcDotQdot_ );

    _PrintDebug(5);    
    if(!task->IsTaskSet()){ printf("1st task is not set!\n"); exit(0); }
    task->getTaskJacobian(Jt);
    task->getTaskJacobianDotQdot(JtDotQdot);
    task->getCommand(xddot);
    dim_first_task_ = task->getDim();

    JtPre = Jt * Npre;
//    dynacore::pretty_print(Jt, std::cout, "Jt");
//    dynacore::pretty_print(JtPre, std::cout, "Jt Pre");
    _WeightedInverse(JtPre, Ainv_, JtPreBar);
    Npre = Npre * (dynacore::Matrix::Identity(num_qdot_, num_qdot_)
            - JtPreBar * JtPre);
    
    _PrintDebug(6);    
    //_CheckNullSpace(Npre);
    // Optimization
    _PrintDebug(7);    
    _OptimizationPreparation();
    // Set equality constraints
    dynacore::Matrix dyn_CE(dim_eq_cstr_, dim_opt_);
    dynacore::Vector dyn_ce0(dim_eq_cstr_);
    dyn_CE.block(0,0, dim_eq_cstr_, dim_first_task_) = Sf_ * A_ * JtPreBar;
    dyn_CE.block(0, dim_first_task_, dim_eq_cstr_, dim_rf_) = -Sf_ * JcN.transpose();
    dyn_ce0 = Sf_ * (A_ * (JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre) + qddot_pre)
            + cori_ + grav_);
    for(int i(0); i< dim_eq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CE[j][i] = dyn_CE(i,j);
        }
        ce0[i] = dyn_ce0[i];
    }
    // Set inequality constraints
    _SetInEqualityConstraint();

    // Optimization
    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);
    dynacore::Vector delta(dim_first_task_);
    for(int i(0); i<dim_first_task_; ++i) { delta[i] = z[i]; }
    qddot_pre = qddot_pre + JtPreBar * (xddot + delta - JtDotQdot - Jt * qddot_pre);

    // First Qddot is found
    // Stack The last Task
    for(int i(1); i<task_list.size(); ++i){
        task = task_list[i];

        if(!task->IsTaskSet()){ printf("%d th task is not set!\n", i); exit(0); }
        task->getTaskJacobian(Jt);
        task->getTaskJacobianDotQdot(JtDotQdot);
        task->getCommand(xddot);

        JtPre = Jt * Npre;
        _WeightedInverse(JtPre, Ainv_, JtPreBar);

        qddot_pre = qddot_pre + JtPreBar * (xddot - JtDotQdot - Jt * qddot_pre);
    
        Npre = Npre * (dynacore::Matrix::Identity(num_qdot_, num_qdot_) 
                - JtPreBar * JtPre);
    }

    _GetSolution(qddot_pre, cmd);

    data_->opt_result_ = dynacore::Vector(dim_opt_);
    for(int i(0); i<dim_opt_; ++i){
        data_->opt_result_[i] = z[i];
    }
       //std::cout << "f: " << f << std::endl;
       //std::cout << "x: " << z << std::endl;
       //std::cout << "cmd: "<<cmd<<std::endl;
    //dynacore::pretty_print(Sf_, std::cout, "Sf");
    //dynacore::pretty_print(qddot_pre, std::cout, "qddot_pre");
    //dynacore::pretty_print(JcN, std::cout, "JcN");
    //dynacore::pretty_print(Nci_, std::cout, "Nci");
    //dynacore::Vector eq_check = dyn_CE * data_->opt_result_;
    //dynacore::pretty_print(dyn_ce0, std::cout, "dyn ce0");
    //dynacore::pretty_print(eq_check, std::cout, "eq_check");
 
    //dynacore::pretty_print(Jt, std::cout, "Jt");
    //dynacore::pretty_print(JtDotQdot, std::cout, "Jtdotqdot");
    //dynacore::pretty_print(xddot, std::cout, "xddot");

   
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


     //if(f > 1.e5){
       //std::cout << "f: " << f << std::endl;
       //std::cout << "x: " << z << std::endl;
       //std::cout << "cmd: "<<cmd<<std::endl;

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
     //}

}

void WBDC_Rotor::_SetInEqualityConstraint(){
    dynacore::Matrix dyn_CI(dim_ieq_cstr_, dim_opt_); dyn_CI.setZero();
    dynacore::Vector dyn_ci0(dim_ieq_cstr_);

    dyn_CI.block(0, dim_first_task_, dim_rf_cstr_, dim_rf_) = Uf_;
    dyn_ci0 = uf_ieq_vec_;
 
   for(int i(0); i< dim_ieq_cstr_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CI[j][i] = dyn_CI(i,j);
        }
        ci0[i] = dyn_ci0[i];
    }
   // dynacore::pretty_print(dyn_CI, std::cout, "WBDC_Rotor: CI");
    // dynacore::pretty_print(dyn_ci0, std::cout, "WBDC_Rotor: ci0");
}

void WBDC_Rotor::_ContactBuilding(const std::vector<ContactSpec*> & contact_list){
    dynacore::Matrix Uf;
    dynacore::Vector uf_ieq_vec;
    // Initial
    dynacore::Matrix Jc;
    dynacore::Vector JcDotQdot;
    contact_list[0]->getContactJacobian(Jc);
    contact_list[0]->getJcDotQdot(JcDotQdot);
    Jc_ = Jc;

    JcDotQdot_ = JcDotQdot;
    static_cast<WBDC_ContactSpec*>(contact_list[0])->getRFConstraintMtx(Uf_);
    static_cast<WBDC_ContactSpec*>(contact_list[0])->getRFConstraintVec(uf_ieq_vec_);

    dim_rf_ = contact_list[0]->getDim();
    dim_rf_cstr_ = 
        static_cast<WBDC_ContactSpec*>(contact_list[0])->getDimRFConstratint();

    int dim_new_rf, dim_new_rf_cstr;

    for(int i(1); i<contact_list.size(); ++i){
        contact_list[i]->getContactJacobian(Jc);
        contact_list[i]->getJcDotQdot(JcDotQdot);
        dim_new_rf = contact_list[i]->getDim();
        dim_new_rf_cstr = static_cast<WBDC_ContactSpec*>(contact_list[i])->getDimRFConstratint();

        // Jc append
        Jc_.conservativeResize(dim_rf_ + dim_new_rf, num_qdot_);
        Jc_ = Jc_.block(dim_rf_, 0, dim_new_rf, num_qdot_) = Jc;

        // JcDotQdot append
        JcDotQdot_.conservativeResize(dim_rf_ + dim_new_rf, 1);
        JcDotQdot_.tail(dim_new_rf) = JcDotQdot;

        // Uf
        static_cast<WBDC_ContactSpec*>(contact_list[i])->getRFConstraintMtx(Uf);
        Uf_.conservativeResize(dim_rf_cstr_ + dim_new_rf_cstr, dim_rf_ + dim_new_rf);
        Uf_.block(0, dim_rf_, dim_rf_cstr_, dim_new_rf).setZero();
        Uf_.block(dim_rf_cstr_, 0, dim_new_rf_cstr, dim_rf_).setZero();
        Uf_.block(dim_rf_cstr_, dim_rf_, dim_new_rf_cstr, dim_new_rf) = Uf;

        // Uf inequality vector
        static_cast<WBDC_ContactSpec*>(contact_list[i])->getRFConstraintVec(uf_ieq_vec);
        uf_ieq_vec_.conservativeResize(dim_rf_cstr_ + dim_new_rf_cstr);
        uf_ieq_vec_.tail(dim_new_rf_cstr) = uf_ieq_vec;

        // Increase reaction force dimension
        dim_rf_ += dim_new_rf;
        dim_rf_cstr_ += dim_new_rf_cstr;
    }
    // printf("dim rf: %i, dim rf constr: %i \n", dim_rf_, dim_rf_cstr_);
    // dynacore::pretty_print(Jc_, std::cout, "WBDC_Rotor: Jc");
    // dynacore::pretty_print(JcDotQdot_, std::cout, "WBDC_Rotor: JcDot Qdot");
    // dynacore::pretty_print(Uf_, std::cout, "WBDC_Rotor: Uf");
}

void WBDC_Rotor::_GetSolution(const dynacore::Vector & qddot, dynacore::Vector & cmd){
    dynacore::Vector Fr(dim_rf_);
    for(int i(0); i<dim_rf_; ++i) Fr[i] = z[i + dim_first_task_];
    dynacore::Vector tot_tau = A_ * qddot + cori_ + grav_ - (Jc_).transpose() * Fr;
    
    cmd = tot_tau.tail(num_act_joint_);

    dynacore::Vector tot_tau_ff = (A_ + data_->A_rotor) * qddot 
        + cori_ + grav_ - (Jc_).transpose() * Fr;
    dynacore::Vector tot_tau_ff_trc = tot_tau_ff.tail(num_qdot_-6);
 
    data_->cmd_ff = tot_tau_ff_trc.tail(num_act_joint_);
    data_->reflected_reaction_force_ = Jc_.transpose() * Fr;
    data_->result_qddot_ = qddot;

    //cmd = (UNci_trc.inverse()).transpose()* tot_tau_trc;
     //dynacore::pretty_print(Jc_, std::cout, "Jc");
     //dynacore::pretty_print(UNci_trc, std::cout, "UNci+trc");
     //dynacore::pretty_print(UNciBar_trc, std::cout, "UNciBar_trc");
     //dynacore::pretty_print(tot_tau_trc, std::cout, "tot tau trc");
    //_WeightedInverse(UNci, Ainv_, UNciBar);
    //cmd = UNciBar.transpose() * tot_tau;
    // dynacore::pretty_print(result, std::cout, "opt result");
     //dynacore::pretty_print(tot_tau, std::cout, "tot tau result");
     //dynacore::pretty_print(cmd, std::cout, "final command");
}

void WBDC_Rotor::_OptimizationPreparation(){
    dim_opt_ = dim_rf_ + dim_first_task_; 
    dim_eq_cstr_ = 6;
    dim_ieq_cstr_ = dim_rf_cstr_; 

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
     for (int i(0); i < dim_opt_; ++i){
        G[i][i] = data_->cost_weight[i];
    }
}

