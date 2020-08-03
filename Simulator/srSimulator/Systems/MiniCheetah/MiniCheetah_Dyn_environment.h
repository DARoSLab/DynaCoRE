#ifndef  DYN_ENVIRONMENT_MiniCheetah
#define  DYN_ENVIRONMENT_MiniCheetah

#include "LieGroup/LieGroup.h"
#include <vector>
#include <Utils/wrap_eigen.hpp>

//TEST JUNHYEOK
#include "MiniCheetah.h"

//TEST
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
class MiniCheetah_Command;
class MiniCheetah_SensorData;

class MiniCheetah_Dyn_environment
{
public:
  MiniCheetah_Dyn_environment();
  ~MiniCheetah_Dyn_environment();

  static void ControlFunction(void* _data);
  void Rendering_Fnc();

  void SetCurrentState_All();
  void saveLandingLocation();
public:
   //MiniCheetah_SensorData* data_;
  //MiniCheetah_Command* cmd_;

 //interface* interface_;
  MiniCheetah* robot_;

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
