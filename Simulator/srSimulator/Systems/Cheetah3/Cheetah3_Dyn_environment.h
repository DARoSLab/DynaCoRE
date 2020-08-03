#ifndef  DYN_ENVIRONMENT_Cheetah3
#define  DYN_ENVIRONMENT_Cheetah3

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

#include "Cheetah3.h"

////////////////////////////////////////////////
#ifdef __APPLE__
#include <GLUT/glut.h>
#endif

//#ifdef linux
#ifdef __linux__
#include <GL/glut.h>
#endif
////////////////////////////////////////////////


class interface;
class srSpace;
class Ground;
class Cheetah3_Command;
class Cheetah3_SensorData;

class Cheetah3_Dyn_environment
{
public:
  Cheetah3_Dyn_environment();
  ~Cheetah3_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
   Cheetah3_SensorData* data_;
  Cheetah3_Command* cmd_;

 interface* interface_;
  Cheetah3* robot_;

  srSpace*	m_Space;
  Ground*	m_ground;

  double ori_mtx_[9];
  std::vector<double> ang_vel_  ;
  void getIMU_Data(std::vector<double> & imu_acc,
          std::vector<double> & imu_ang_vel);
 protected:
  void _Get_Orientation(dynacore::Quaternion & rot);
  void _Copy_Array(double * , double *, int);
  void _CheckFootContact(bool & r_contact, bool & l_contact);
};

#endif
