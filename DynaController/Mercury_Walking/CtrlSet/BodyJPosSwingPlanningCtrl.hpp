#ifndef BODY_JOINT_POSITION_FOOT_WALKING_CONTROL
#define BODY_JOINT_POSITION_FOOT_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Utils/BSplineBasic.h>
#include <Mercury_Controller/StateEstimator/LIPM_KalmanFilter.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>
#include <Utils/minjerk_one_dim.hpp>

class BodyJPosSwingPlanningCtrl:public SwingPlanningCtrl{
    public:
        BodyJPosSwingPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        virtual ~BodyJPosSwingPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        dynacore::Vect3 curr_jpos_des_;
        dynacore::Vect3 curr_jvel_des_;
        dynacore::Vect3 curr_jacc_des_;

    protected:
        void _SetBspline(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);

        // JPos B- spline
        void _SetBspline(const dynacore::Vector & guess_q,
                const dynacore::Vect3 & st_config,
                const dynacore::Vect3 & st_jvel,
                const dynacore::Vect3 & st_jacc,
                const dynacore::Vect3 & target_pos);

        dynacore::Vector ini_swing_leg_config_;
        dynacore::Vector mid_swing_leg_config_;
        dynacore::Vector target_swing_leg_config_;

        bool b_initial_planning_;
        double kp_x_;
        double kp_y_;


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
        void _Replanning(dynacore::Vect3 & target_loc);
        void _getHurstPlan(dynacore::Vect3 & target_loc);

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;
        dynacore::Vect2 body_pt_offset_;

        dynacore::Vect2 prev_ekf_vel;
        dynacore::Vect2 acc_err_ekf;
        
        dynacore::Vector ini_config_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;
        BS_Basic<3, 3, 1, 2, 2> swing_leg_traj_;
        std::vector<MinJerk_OneDimension*> min_jerk_cartesian;
        void _SetCartesianMinJerk(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);


        double swing_time_reduction_;

        void _task_setup();
        void _single_contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        Mercury_InvKinematics inv_kin_;
        double ctrl_start_time_;
};

#endif
