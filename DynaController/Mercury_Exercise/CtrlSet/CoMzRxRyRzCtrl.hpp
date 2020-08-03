#ifndef COM_HEIGHT_RX_RY_RZ_CONTROL
#define COM_HEIGHT_RX_RY_RZ_CONTROL

#include <Controller.hpp>

class Mercury_StateProvider;
class WBDC_Relax;
class WBDC_Relax_ExtraData;
class WBDC_Relax_Task;
class WBDC_ContactSpec;
class RobotSystem;

class WBDC_Rotor;
class WBDC_Rotor_ExtraData;

class CoMzRxRyRzCtrl: public Controller{
    public:
        CoMzRxRyRzCtrl(RobotSystem*);
        virtual ~CoMzRxRyRzCtrl();

        virtual void OneStep(dynacore::Vector & gamma);
        virtual void FirstVisit();
        virtual void LastVisit();
        virtual bool EndOfPhase();

        virtual void CtrlInitialization(const std::string & setting_file_name);

        void setStanceTime(double stance_time){ end_time_ = stance_time; }
        void setAmp(const std::vector<double> & amp);
        void setFrequency(const std::vector<double> & freq);
        void setPhase(const std::vector<double> & phase);

        void setStanceHeight(double height) {
            des_com_height_ = height;
            b_set_height_target_ = true;
        }

    protected:
        dynacore::Vector amp_, freq_, phase_;

        int dim_ctrl_;
        bool b_set_height_target_;
        double des_com_height_;

        WBDC_Relax* wbdc_;
        WBDC_Relax_ExtraData* wbdc_data_;
        WBDC_Relax_Task* com_task_;
        WBDC_ContactSpec* double_contact_;

        WBDC_Rotor* wbdc_rotor_;
        WBDC_Rotor_ExtraData* wbdc_rotor_data_;

        dynacore::Vect3 ini_com_pos_;

        double end_time_;
        void _com_task_setup();
        void _double_contact_setup();
        void _com_ctrl(dynacore::Vector & gamma);
        void _com_ctrl_wbdc_rotor(dynacore::Vector & gamma);

        Mercury_StateProvider* sp_;
        double ctrl_start_time_;
};

#endif
