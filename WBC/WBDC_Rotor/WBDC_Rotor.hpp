/* Whole-body dynamic control account rotor inertia:
 * No internal constrain */

#ifndef WHOLE_BODY_DYNAMIC_CONTROL_ROTOR_INERTIA_H
#define WHOLE_BODY_DYNAMIC_CONTROL_ROTOR_INERTIA_H

#include <WBC.hpp>
#include <Utils/utilities.hpp>
#include <Optimizer/Goldfarb/QuadProg++.hh>

#include <Task.hpp>
#include <WBDC/WBDC_ContactSpec.hpp>

class WBDC_Rotor_ExtraData{
    public:
        dynacore::Vector cost_weight;
        dynacore::Matrix A_rotor;

        dynacore::Vector cmd_ff; //Feedforward torque cmd
        dynacore::Vector opt_result_;

        dynacore::Vector result_qddot_;
        dynacore::Vector reflected_reaction_force_;
        WBDC_Rotor_ExtraData(){}
        ~WBDC_Rotor_ExtraData(){}
};

class WBDC_Rotor: public WBC{
    public:
        WBDC_Rotor(const std::vector<bool> & act_list, const dynacore::Matrix* Jci = NULL);
        virtual ~WBDC_Rotor(){}

        virtual void UpdateSetting(const dynacore::Matrix & A,
                const dynacore::Matrix & Ainv,
                const dynacore::Vector & cori,
                const dynacore::Vector & grav,
                void* extra_setting = NULL);

        virtual void MakeTorque(const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                dynacore::Vector & cmd,
                void* extra_data = NULL);

    private:
        void _SetInEqualityConstraint();
        void _ContactBuilding(const std::vector<ContactSpec*> & contact_list);

        void _GetSolution(const dynacore::Vector & qddot, dynacore::Vector & cmd);
        bool _CheckNullSpace(const dynacore::Matrix & Npre);
        void _OptimizationPreparation();

        int dim_opt_;
        int dim_eq_cstr_; // equality constraints
        int dim_ieq_cstr_; // inequality constraints
        int dim_first_task_; // first task dimension
        WBDC_Rotor_ExtraData* data_;

        GolDIdnani::GVect<double> z;
        // Cost
        GolDIdnani::GMatr<double> G;
        GolDIdnani::GVect<double> g0;

        // Equality
        GolDIdnani::GMatr<double> CE;
        GolDIdnani::GVect<double> ce0;

        // Inequality
        GolDIdnani::GMatr<double> CI;
        GolDIdnani::GVect<double> ci0;

        int dim_rf_;
        int dim_relaxed_task_;
        int dim_cam_;
        int dim_rf_cstr_;

        dynacore::Matrix tot_tau_Mtx_;
        dynacore::Vector tot_tau_Vect_;

        dynacore::Matrix S_delta_;
        dynacore::Matrix Uf_;
        dynacore::Vector uf_ieq_vec_;

        dynacore::Matrix Jc_;
        dynacore::Vector JcDotQdot_;

        dynacore::Matrix B_;
        dynacore::Vector c_;
        dynacore::Vector task_cmd_;

        dynacore::Matrix Sf_; //floating base
        void _PrintDebug(double i) {
            //printf("[WBDC_Rotor] %f \n", i);
        }
};

#endif
