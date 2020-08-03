#ifndef CONFIGURATION_BODY_FOOT_WALKING_CONTROL
#define CONFIGURATION_BODY_FOOT_WALKING_CONTROL

#include <Controller.hpp>
#include <Utils/BSplineBasic.h>
#include <Mercury_Controller/StateEstimator/LIPM_KalmanFilter.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>

class Mercury_StateProvider;
class RobotSystem;
class Planner;

class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class ConfigBodyFootPlanningCtrl:public Controller{
    public:
        ConfigBodyFootPlanningCtrl(RobotSystem* robot, int swing_foot, Planner* planner);
        ~ConfigBodyFootPlanningCtrl();
        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setPlanningFrequency(double freq){ planning_frequency_ = freq; }
        void setSwingTime(double swing_time){ end_time_ = swing_time; }
        void setDoubleStanceRatio(double ratio){ double_stance_ratio_ = ratio;}
        void setTransitionPhaseRatio(double ratio){ transition_phase_ratio_ = ratio;}

        void notifyTransitionTime(double time){ transition_time_ = time; }
        void notifyStanceTime(double time){ stance_time_ = time; }

        void setPrimeTimeX(double t_p_x){ t_prime_x_ = t_p_x; }
        void setPrimeTimeY(double t_p_y){ t_prime_y_ = t_p_y; }
        void setStanceHeight(double height) {
            des_body_height_ = height;
            b_set_height_target_ = true;
            // CoM estimator
            com_estimator_->h_ = des_body_height_;
        }
        void setContactSwitchCheck(bool switch_check){ b_contact_switch_check_ = switch_check; }

        dynacore::Vect3 curr_foot_pos_des_;
        dynacore::Vect3 curr_foot_vel_des_;
        dynacore::Vect3 curr_foot_acc_des_;

    protected:
        void _CoMEstiamtorUpdate();

        void _SetBspline(
                const dynacore::Vect3 & st_pos,
                const dynacore::Vect3 & st_vel,
                const dynacore::Vect3 & st_acc,
                const dynacore::Vect3 & target_pos);

        double replan_moment_;

        bool b_contact_switch_check_;
        bool b_set_height_target_;
        double des_body_height_;

        double double_stance_ratio_;
        double transition_phase_ratio_;

        int swing_foot_;
        double swing_height_;
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;
    
        double planning_frequency_;
        int num_planning_;
        double t_prime_x_;
        double t_prime_y_;

        dynacore::Vector task_kp_;
        dynacore::Vector task_kd_;

        double gain_decreasing_ratio_;
        double gain_decreasing_period_portion_;
        void _setTaskGain(const dynacore::Vector & Kp, const dynacore::Vector & Kd);

        Task* config_body_foot_task_;
        WBDC_ContactSpec* single_contact_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        Planner* planner_;
        void _CheckPlanning();
        void _Replanning();

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Vect3 target_foot_pos_;
        
        dynacore::Vector ini_config_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;

        double end_time_;
        double transition_time_;
        double stance_time_;

        void _task_setup();
        void _single_contact_setup();
        void _body_foot_ctrl(dynacore::Vector & gamma);

        CoMStateEstimator* com_estimator_;
        Mercury_StateProvider* sp_;
        Mercury_InvKinematics inv_kin_;
        double ctrl_start_time_;
};

#endif
