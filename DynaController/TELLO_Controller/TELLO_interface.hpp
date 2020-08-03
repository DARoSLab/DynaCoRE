#ifndef TELLO_INTERFACE_H
#define TELLO_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "TELLO_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class TELLO_StateEstimator;
class TELLO_StateProvider;

class TELLO_interface: public interface{
    public:
        TELLO_interface();
        virtual ~TELLO_interface();
        virtual void GetCommand(void * data, void * command);

    private:
        int waiting_count_;

        void _ParameterSetting();
        bool _Initialization(TELLO_SensorData* );

        TELLO_Command* test_cmd_;
        dynacore::Vector initial_upper_body_config_;

        dynacore::Vector jjvel_;
        dynacore::Vector jjpos_;
        dynacore::Vector jtorque_;

        dynacore::Vector torque_command_;
        dynacore::Vector jpos_command_;
        dynacore::Vector jvel_command_;

        TELLO_StateEstimator* state_estimator_;
        TELLO_StateProvider* sp_;
};

#endif
