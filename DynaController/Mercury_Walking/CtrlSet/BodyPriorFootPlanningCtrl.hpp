#ifndef BODY_PRIOR_FOOT_CONFIGURATION_WALKING_CONTROL
#define BODY_PRIOR_FOOT_CONFIGURATION_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Utils/BSplineBasic.h>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

class BodyPriorFootPlanningCtrl:public SwingPlanningCtrl{
    public:
        BodyPriorFootPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        ~BodyPriorFootPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

    protected:
        void _CoMEstiamtorUpdate();

        void _SetBspline(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);

        int swing_leg_jidx_;
        int stance_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;
    
        dynacore::Vector task_kp_;
        dynacore::Vector task_kd_;

        double gain_decreasing_ratio_;
        double gain_decreasing_period_portion_;

        Task* stance_task_;
        Task* jpos_swing_task_;

        void _CheckPlanning();
        void _Replanning();

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;
        
        dynacore::Vector ini_config_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;

        void _task_setup();
        void _single_contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        Mercury_InvKinematics inv_kin_;
        double ctrl_start_time_;
};

#endif
