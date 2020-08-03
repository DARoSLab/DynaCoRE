#ifndef MERCURY_INTERFACE_H
#define MERCURY_INTERFACE_H

#include <Utils/wrap_eigen.hpp>
#include <Configuration.h>
#include <interface.hpp>
#include <Mercury/Mercury_Definition.h>

class Mercury_StateEstimator;
class Mercury_StateProvider;

class Mercury_SensorData{
    public:
        double imu_inc[3];
        double imu_ang_vel[3];
        double imu_acc[3];
        double joint_jpos[mercury::num_act_joint];
        double joint_jvel[mercury::num_act_joint];
        double motor_jpos[mercury::num_act_joint];
        double motor_jvel[mercury::num_act_joint];
        double bus_current[mercury::num_act_joint];
        double bus_voltage[mercury::num_act_joint];
        double jtorque[mercury::num_act_joint];
        double motor_current[mercury::num_act_joint];
        double reflected_rotor_inertia[mercury::num_act_joint];
        bool rfoot_contact;
        bool lfoot_contact;
};

class Mercury_interface: public interface{
public:
  Mercury_interface();
  virtual ~Mercury_interface();

public:
  virtual void GetCommand(void * data,
                  std::vector<double> & command);
  void GetReactionForce(std::vector<dynacore::Vect3> & reaction_force );

  dynacore::Quaternion global_ori_;

private:
  int waiting_count_;
  std::vector<double> torque_limit_min_;
  std::vector<double> torque_limit_max_;
  
  void _ParameterSetting();
  bool _Initialization(Mercury_SensorData* );

  dynacore::Vector virtual_sensor_;
  dynacore::Vector torque_command_;
  dynacore::Vector test_command_;
  dynacore::Vector sensed_torque_;
  dynacore::Vector motor_current_;
  dynacore::Vector bus_current_;
  dynacore::Vector bus_voltage_;

  dynacore::Vector jjvel_;
  dynacore::Vector mjpos_;
  
  dynacore::Vector initial_jpos_;
  Mercury_StateEstimator* state_estimator_;
  Mercury_StateProvider* sp_;
};

#endif
