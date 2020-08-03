#ifndef JOINT_POSITION_DOUBLE_SUPPORT_CONTACT_TRANSITION_CTRL
#define JOINT_POSITION_DOUBLE_SUPPORT_CONTACT_TRANSITION_CTRL

#include <Controller.hpp>
#include <Mercury_Controller/Mercury_DynaControl_Definition.h>

class Mercury_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class JPosDoubleTransCtrl: public Controller{
    public:
        JPosDoubleTransCtrl(RobotSystem* );
        virtual ~JPosDoubleTransCtrl();

        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setMovingTime(double stance_time){ end_time_ = stance_time; }
        
        void setInitialPosition(const std::vector<double> & jpos){
            for(int i(0); i<mercury::num_act_joint; ++i) jpos_ini_[i] = jpos[i];
        }
        void setTargetPosition(const dynacore::Vector & jpos){ 
            b_jpos_set_ = true;
            jpos_target_ = jpos; }


    protected:
        bool b_jpos_set_;
        dynacore::Vector jpos_target_;
        dynacore::Vector jpos_ini_;

        double max_rf_z_;
        double min_rf_z_;

        Task* body_task_;
        WBDC_ContactSpec* double_contact_;
        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector config_des_;

        double end_time_;
        void _body_task_setup();
        void _double_contact_setup();
        void _body_ctrl_wbdc_rotor(dynacore::Vector & gamma);
        double ctrl_start_time_;
        Mercury_StateProvider* sp_;
};

#endif
