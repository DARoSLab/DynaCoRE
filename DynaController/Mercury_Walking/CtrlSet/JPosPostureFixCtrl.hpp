#ifndef JOINT_POSTURE_CTRL
#define JOINT_POSTURE_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class WBWC;

class JPosPostureFixCtrl: public Controller{
    public:
        JPosPostureFixCtrl(RobotSystem* );
        virtual ~JPosPostureFixCtrl();

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
        WBWC* wbwc_;
        double end_time_;

        Task* jpos_task_;
        WBDC_ContactSpec* contact_constraint_;
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
        void _contact_constraint_setup();
        void _jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
};

#endif
