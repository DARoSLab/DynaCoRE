#ifndef JOINT_POSITION_SWING_CONTROL
#define JOINT_POSITION_SWING_CONTROL

#include <Controller.hpp>
#include <Utils/BSplineBasic.h>
#include <Utils/minjerk_one_dim.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

class Planner;
class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;
class WBWC;

class JPosSwingCtrl:public Controller{
    public:
        JPosSwingCtrl(const RobotSystem* robot, int swing_foot, Planner* planner);
        virtual ~JPosSwingCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();
        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double time) { end_time_ = time; }
        void setReplanning(bool on){ b_replan_ = on; }
 
        void setDoubleStanceRatio(double ratio){  double_stance_ratio_ = ratio;}
        void setTransitionPhaseRatio(double ratio){  transition_phase_ratio_ = ratio;}

        void notifyTransitionTime(double time){  transition_time_ = time; }
        void notifyStanceTime(double time){  stance_time_ = time; }

        void setContactSwitchCheck(bool switch_check){ 
            b_contact_switch_check_ = switch_check; }

        void setStancePosture(const dynacore::Vector& stance_jpos){
            stance_jpos_ = stance_jpos;
        }

        dynacore::Vect3 curr_foot_pos_des_;
        dynacore::Vect3 curr_foot_vel_des_;
        dynacore::Vect3 curr_foot_acc_des_;

    protected:
        double abduction_addition_;
        double hip_addition_;

        double abduction_addition_replan_;
        double hip_addition_replan_;

        dynacore::Vector prev_jpos_des_;

        WBWC* wbwc_;
        double planning_moment_portion_;
        bool b_replan_;
        bool b_planned_;

        dynacore::Vect3 body_pt_offset_;
        double pitch_offset_gain_;
        double roll_offset_gain_;

        dynacore::Vect3 noplan_landing_loc_;

        dynacore::Vector swing_jpos_addition_;
        dynacore::Vector swing_jpos_replanned_addition_;
        dynacore::Vector target_swing_leg_config_;
        dynacore::Vector ini_config_;

        dynacore::Vector stance_jpos_;
        dynacore::Vector jpos_swing_delta_;

        int swing_leg_jidx_;
        int stance_leg_jidx_;
        bool b_contact_switch_check_;
        bool b_set_height_target_;
        double des_body_height_;

        double double_stance_ratio_;
        double transition_phase_ratio_;
        double replan_moment_;
        double transition_time_;
        double stance_time_;

        int swing_foot_;
        double swing_height_;
        dynacore::Vect3 default_target_loc_;

        std::vector<MinJerk_OneDimension*> min_jerk_jpos_initial_;
        double end_time_;

        double push_down_height_;

        Task* jpos_task_;
        WBDC_ContactSpec* contact_constraint_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        double swing_time_reduction_;

        void _task_setup();
        void _contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        void _SetMinJerkTraj(
                double moving_duration,
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos,
                const dynacore::Vect3 & target_vel,
                const dynacore::Vect3 & target_acc);
        void _Replanning(dynacore::Vect3 & target_loc);
        void _CheckPlanning();

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        Mercury_StateProvider* sp_;
        double ctrl_start_time_;
        Planner* planner_;
        Mercury_InvKinematics inv_kin_;
};

#endif
