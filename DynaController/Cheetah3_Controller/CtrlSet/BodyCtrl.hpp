#ifndef BODY_CTRL_Cheetah3
#define BODY_CTRL_Cheetah3

#include <Controller.hpp>

class Cheetah3_StateProvider;
class RobotSystem;
class WBDC_ContactSpec;
class WBLC;
class WBLC_ExtraData;
class KinWBC;

class BodyCtrl: public Controller{
    public:
        BodyCtrl(RobotSystem* );
        virtual ~BodyCtrl();

        virtual void OneStep(void* _cmd);
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
        dynacore::Vector Kp_, Kd_;
        dynacore::Vector des_jpos_; 
        dynacore::Vector des_jvel_; 
        dynacore::Vector des_jacc_;

        dynacore::Vector jpos_ini_;
        bool b_set_height_target_;
        double end_time_;
        int dim_contact_;

        std::vector<int> selected_jidx_;
        Task* total_joint_task_;
        Task* body_pos_task_; //pelvis
        Task* body_ori_task_;
        Task* selected_joint_task_;
        KinWBC* kin_wbc_;

        WBDC_ContactSpec* fr_contact_;
        WBDC_ContactSpec* fl_contact_;
        WBDC_ContactSpec* hr_contact_;
        WBDC_ContactSpec* hl_contact_;

        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        double target_body_height_;
        double ini_body_height_;
        dynacore::Vect3 ini_body_pos_;

        void _task_setup();
        void _contact_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);

        double ctrl_start_time_;
        Cheetah3_StateProvider* sp_;
};

#endif
