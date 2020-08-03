#ifndef JOINT_POSITION_SINGLE_TRANSITION_CTRL
#define JOINT_POSITION_SINGLE_TRANSITION_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;
class WBWC;

class JPosSingleTransCtrl: public Controller{
    public:
        JPosSingleTransCtrl(RobotSystem* robot, int moving_foot, bool b_increase);
        virtual ~JPosSingleTransCtrl();

        virtual void OneStep(void* cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double time) { end_time_ = time; }
        void setPosture(const dynacore::Vector & set_jpos){
            set_jpos_ = set_jpos;
            b_jpos_set_ = true;
        }

    protected:
        double right_knee_add_;
        double left_knee_add_;

        double initial_rknee_jpos_des_;
        double initial_lknee_jpos_des_;


        int stance_leg_jidx_;
        int swing_foot_;
        WBWC* wbwc_;
        double end_time_;
        double max_rf_z_;
        double min_rf_z_;
        bool b_increase_;

        Task* jpos_task_;
        WBDC_ContactSpec* contact_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vector jpos_ini_;
        dynacore::Vector jpos_target_;
        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        bool b_jpos_set_;
        dynacore::Vector set_jpos_;

        void _jpos_task_setup();
        void _contact_setup();
        void _jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
};

#endif
