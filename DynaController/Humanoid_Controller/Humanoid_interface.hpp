#ifndef Humanoid_INTERFACE_H
#define Humanoid_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include "Humanoid_DynaCtrl_Definition.h"
#include <Filter/filters.hpp>

class Humanoid_StateEstimator;
class Humanoid_StateProvider;

class Humanoid_interface: public interface{
    public:
        Humanoid_interface();
        virtual ~Humanoid_interface();
        virtual void GetCommand(void * data, void * command);

    private:
        int waiting_count_;

        void _ParameterSetting();
        bool _Initialization(Humanoid_SensorData* );

        Humanoid_Command* test_cmd_;
        dynacore::Vector initial_upper_body_config_;

        dynacore::Vector jjvel_;
        dynacore::Vector jjpos_;
        dynacore::Vector jtorque_;

        dynacore::Vector torque_command_;
        dynacore::Vector jpos_command_;
        dynacore::Vector jvel_command_;

        Humanoid_StateEstimator* state_estimator_;
        Humanoid_StateProvider* sp_;
};

#endif
