#ifndef FOOT_Swing_SagitP3
#define FOOT_Swing_SagitP3

#include <Controller.hpp>
#include <Utils/minjerk_one_dim.hpp>
#include <Utils/BSplineBasic.h>
#include <SagitP3_Controller/SagitP3_StateProvider.hpp>
#include <SagitP3_Controller/SagitP3_DynaCtrl_Definition.h>

class KinWBC;
class WBDC_ContactSpec;
class WBLC;
class WBLC_ExtraData;

class FootSwingCtrl:public Controller{
   public:
        FootSwingCtrl(const RobotSystem* robot, int swing_foot);
        virtual ~FootSwingCtrl();
        virtual void OneStep(void* _cmd);
        virtual void FirstVisit();
        virtual void LastVisit(){ sp_->des_jpos_prev_ = des_jpos_; }
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);
        void setSwingTime(double swing_time){ 
            end_time_ = swing_time; 
            half_swing_time_ = swing_time/2.;
        }
        void setStanceHeight(double height) {
            des_body_height_ = height;
            b_set_height_target_ = true;
        }
        dynacore::Vect3 curr_foot_pos_des_;
        dynacore::Vect3 curr_foot_vel_des_;
        dynacore::Vect3 curr_foot_acc_des_;


    protected:
        double ini_base_height_;
        int swing_leg_jidx_;
        double push_down_height_; // push foot below the ground at landing
        dynacore::Vect3 default_target_loc_;
        dynacore::Vect3 initial_target_loc_;
        
        int dim_contact_;
        WBDC_ContactSpec* rfoot_contact_;
        WBDC_ContactSpec* lfoot_contact_;

        void _contact_setup();
        void _task_setup();
        void _compute_torque_wblc(dynacore::Vector & gamma);
        void _SetMinJerkOffset(const dynacore::Vect3 & offset);
        void _SetBspline(
            const dynacore::Vect3 & st_pos, 
            const dynacore::Vect3 & des_pos);

        void _GetSinusoidalSwingTrajectory();
        void _GetBsplineSwingTrajectory();
        std::vector<ContactSpec*> kin_wbc_contact_list_;
        
        Task* base_task_;
        Task* foot_task_;
        KinWBC* kin_wbc_;
        WBLC* wblc_;
        WBLC_ExtraData* wblc_data_;

        SagitP3_StateProvider* sp_;

        int swing_foot_;
        double swing_height_;
        bool b_set_height_target_;
        double des_body_height_;

        dynacore::Vector des_jpos_;
        dynacore::Vector des_jvel_;
        dynacore::Vector des_jacc_;

        dynacore::Vector Kp_;
        dynacore::Vector Kd_;

        dynacore::Vect3 ini_body_pos_;
        dynacore::Vect3 ini_com_pos_;
        dynacore::Vect3 ini_foot_pos_;
        dynacore::Quaternion ini_foot_ori_;
        dynacore::Vect2 body_pt_offset_;
        
        dynacore::Vector ini_config_;

        std::vector<double> foot_landing_offset_;

        std::vector<MinJerk_OneDimension*> min_jerk_offset_;
        BS_Basic<3, 3, 1, 2, 2> foot_traj_;

        double end_time_;
        double half_swing_time_;
        double transition_time_;
        double stance_time_;
        double ctrl_start_time_;
};

#endif
