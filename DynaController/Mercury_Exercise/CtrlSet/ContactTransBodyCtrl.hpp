#ifndef CONTACT_TRANSITION_BODY_CTRL
#define CONTACT_TRANSITION_BODY_CTRL

#include <Controller.hpp>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class ContactTransBodyCtrl: public Controller{
    public:
        ContactTransBodyCtrl(RobotSystem* );
        virtual ~ContactTransBodyCtrl();

        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double stance_time){ end_time_ = stance_time; }
        void setStanceHeight(double height) {
            des_body_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        bool b_set_height_target_;
        double des_body_height_;
        double max_rf_z_;
        double min_rf_z_;

        Task* body_task_;
        WBDC_ContactSpec* double_contact_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vector body_pos_ini_;
        dynacore::Vect3 ini_body_pos_;

        double end_time_;
        void _body_task_setup();
        void _double_contact_setup();
        void _body_ctrl_wbdc_rotor(dynacore::Vector & gamma);
        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
};

#endif
