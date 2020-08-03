#ifndef Cheetah3_INTERFACE_H
#define Cheetah3_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "Cheetah3_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class Cheetah3_StateEstimator;
class Cheetah3_StateProvider;

class Cheetah3_interface: public interface{
    public:
        Cheetah3_interface();
        virtual ~Cheetah3_interface();
        virtual void GetCommand(void * data, void * command);

    private:
        int waiting_count_;

        void _ParameterSetting();
        bool _Initialization(Cheetah3_SensorData* );

        Cheetah3_Command* test_cmd_;
        dynacore::Vector initial_upper_body_config_;

        dynacore::Vector jjvel_;
        dynacore::Vector jjpos_;
        dynacore::Vector jtorque_;

        dynacore::Vector torque_command_;
        dynacore::Vector jpos_command_;
        dynacore::Vector jvel_command_;

        Cheetah3_StateEstimator* state_estimator_;
        Cheetah3_StateProvider* sp_;
};

#endif
