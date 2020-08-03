#ifndef JOINT_POSITION_TRAJECTORY_WALKING_CONTROL
#define JOINT_POSITION_TRAJECTORY_WALKING_CONTROL

#include "SwingPlanningCtrl.hpp"
#include <Mercury_Controller/StateEstimator/LIPM_KalmanFilter.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>
#include <Utils/minjerk_one_dim.hpp>

class Mercury_StateProvider;
class WBWC;

class JPosTrajPlanningCtrl:public SwingPlanningCtrl{
    public:
        JPosTrajPlanningCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        virtual ~JPosTrajPlanningCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit(){}
        virtual bool EndOfPhase();
        virtual void CtrlInitialization(const std::string & setting_file_name);

    protected:
        double pitch_offset_gain_;
        double roll_offset_gain_;

        void _SetMinJerkTraj(
                double moving_duration,
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos,
                const dynacore::Vect3 & target_vel,
                const dynacore::Vect3 & target_acc);


        dynacore::Vector target_swing_leg_config_;
        dynacore::Vector prev_jpos_des_;

        int swing_leg_jidx_;
        int stance_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;

        void _CheckPlanning();
        void _Replanning(dynacore::Vect3 & target_loc);
        void _task_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);


        WBWC* wbwc_;

        dynacore::Vect3 adjust_jpos_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect2 body_pt_offset_;

        dynacore::Vect2 prev_ekf_vel;
        dynacore::Vect2 acc_err_ekf;
        
        dynacore::Vector ini_config_;
        std::vector<MinJerk_OneDimension*> min_jerk_jpos_;
        dynacore::Vector swing_jpos_delta_;

        Mercury_InvKinematics inv_kin_;
        double ctrl_start_time_;
};

#endif
