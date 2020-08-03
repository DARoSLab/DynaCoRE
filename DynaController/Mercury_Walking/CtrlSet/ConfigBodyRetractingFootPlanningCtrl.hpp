#ifndef CONFIGURATION_RETRACTING_FOOT_WALKING_CONTROL
#define CONFIGURATION_RETRACTING_FOOT_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Utils/BSplineBasic.h>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>
#include <Utils/minjerk_one_dim.hpp>

class ConfigBodyRetractingFootPlanningCtrl:public SwingPlanningCtrl{
   public:
        ConfigBodyRetractingFootPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        ~ConfigBodyRetractingFootPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
    protected:
        void _SetBspline(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);

        void _SetCartesianMinJerk(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);

        void _SetRetractionMinJerk(
                const dynacore::Vect2 & st_pos,
                const dynacore::Vect2 & st_vel,
                const dynacore::Vect2 & st_acc,
                const dynacore::Vect2 & target_pos);



        double swing_time_reduction_;

        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;
    
        dynacore::Vector task_kp_;
        dynacore::Vector task_kd_;

        double gain_decreasing_ratio_;
        double gain_decreasing_period_portion_;
        void _setTaskGain(const dynacore::Vector & Kp, const dynacore::Vector & Kd);

        Task* config_body_foot_task_;
        void _CheckPlanning();
        void _Replanning();

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;
        dynacore::Vect2 body_pt_offset_;
        
        dynacore::Vector ini_config_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;
        // BS_Basic<4, 4, 1, 3, 3> foot_traj_;

        std::vector<double> foot_landing_offset_;

        void _task_setup();
        void _single_contact_setup();

        void _retract_foot_ctrl(dynacore::Vector & gamma);       
        void _body_foot_ctrl(dynacore::Vector & gamma);

        Mercury_InvKinematics inv_kin_;
        std::vector<MinJerk_OneDimension*> min_jerk_cartesian;
        std::vector<MinJerk_OneDimension*> min_jerk_leg_hip_knee;  
        dynacore::Vect2 hip_knee_desired_retraction;       
        bool gen_min_jerk_stepping_plan;

};

#endif
