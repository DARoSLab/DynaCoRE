#ifndef BODY_JOINT_POSITION_CTRL
#define BODY_JOINT_POSITION_CTRL

#include <Controller.hpp>
#include <Mercury_Controller/Mercury_InvKinematics.hpp>


class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;


class BodyJPosCtrl: public Controller{
    public:
        BodyJPosCtrl(RobotSystem* );
        virtual ~BodyJPosCtrl();

        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double time) { end_time_ = time; }
        void setStanceHeight(double height){ 
            target_body_height_ = height;
            b_set_height_target_ = true;
         }

    protected:
        bool b_set_height_target_;
        int trj_type_;
        double end_time_;

        Task* jpos_task_;
        WBDC_ContactSpec* double_body_contact_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vector jpos_ini_;
        dynacore::Vector jpos_target_;

        double target_body_height_;
        double ini_body_height_;

        bool b_jpos_set_;

        void _jpos_task_setup();
        void _double_body_contact_setup();
        void _jpos_ctrl_wbdc_rotor(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
        Mercury_InvKinematics inv_kin_;
        
};

#endif
