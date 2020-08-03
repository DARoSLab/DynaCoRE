#include "MoCapManager.hpp"
#include <Utils/comm_udp.hpp>
#include "Mercury_StateProvider.hpp"
#include <Utils/utilities.hpp>
#include <Utils/DataManager.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

MoCapManager::MoCapManager(RobotSystem* robot):dynacore_pThread(),
                             socket_(0),
                             mocap_data_(NUM_MARKERS),
                             led_pos_data_(3*NUM_MARKERS),
                             led_kin_data_(3*NUM_MARKERS),
                             led_pos_raw_data_(3*NUM_MARKERS),
                             body_quat_(1.0, 0., 0., 0. )
{
robot_sys_ = robot;
  marker_cond_.resize(NUM_MARKERS, 0.);
  for(int i(0); i<NUM_MARKERS; ++i){
    mocap_data_[i].setZero();
  }

  led_pos_data_.setZero();
  led_kin_data_.setZero();
  //DataManager::GetDataManager()->RegisterData(&led_pos_data_, DYN_VEC, "LED_Pos", 3*NUM_MARKERS);
  //DataManager::GetDataManager()->RegisterData(&led_kin_data_, DYN_VEC, "LED_Kin_Pos", 3*NUM_MARKERS);
  //DataManager::GetDataManager()->RegisterData(&led_pos_raw_data_, DYN_VEC, "LED_Pos_Raw", 3*NUM_MARKERS);

  sp_ = Mercury_StateProvider::getStateProvider();
  
  printf("[Mo Cap Manager] Constructed\n");
}

void MoCapManager::run(){
  message msg;
  mercury_message mercury_msg;
  int count(0);


  while(true){
    ++count;
    COMM::receive_data(socket_, MOCAP_DATA_PORT, &mercury_msg, sizeof(mercury_message), IP_ADDRESS);

    for(int i(0);i<NUM_MARKERS; ++i){
      for(int j(0); j<3; ++j){
        mocap_data_[i][j] = mercury_msg.data[3*i+j];
        led_pos_raw_data_[3*i + j] = mercury_msg.data[3*i+j];
      }
      marker_cond_[i] = mercury_msg.visible[i];
    }
    _CoordinateChange(mercury_msg);
    _UpdateLEDPosData(mercury_msg);

    //if(count% 500 == 0){ _print_message(mercury_msg); }
  }
}

void MoCapManager::_CoordinateChange(mercury_message & msg) {
    double start_idx = mercury_link::LED_BODY_0;
    double len_to_virtual = 77;
    std::vector<dynacore::Vect3> led_list;
    led_list.resize(NUM_MARKERS);
    for (int i = 0; i < NUM_MARKERS; ++i) {
        led_list[i][0] = msg.data[3*i];
        led_list[i][1] = msg.data[3*i + 1];
        led_list[i][2] = msg.data[3*i + 2];
    }
    dynacore::Matrix R(3,3);
    R = _GetOrientation(led_list[mercury_link::LED_BODY_0-start_idx],
                        led_list[mercury_link::LED_BODY_1-start_idx],
                        led_list[mercury_link::LED_BODY_2-start_idx]);

    dynacore::Vect3 p;
    dynacore::Vect3 local_pos;
    p << len_to_virtual, 0., 0.;
    for (int i = 0; i < NUM_MARKERS; ++i) {
      local_pos = R*(led_list[i] - led_list[0]) + p;

      msg.data[3*i] = local_pos[0];
      msg.data[3*i + 1] = local_pos[1];
      msg.data[3*i + 2] = local_pos[2];
    }
}

dynacore::Matrix MoCapManager::_GetOrientation(const dynacore::Vect3 &b0,
                                             const dynacore::Vect3 &b1,
                                             const dynacore::Vect3 &b2) {
    dynacore::Vect3 normal;
    //normal = (b2 - b0).cross( (b1 - b0) );
    normal = (b1 - b0).cross( (b2 - b0) );
    normal /= sqrt(normal[0]*normal[0] + normal[1]*normal[1] +
                    normal[2] * normal[2]);
    dynacore::Vect3 x_coord, y_coord, z_coord;
    x_coord = normal;
    // Y
    y_coord = b1-b2;
    y_coord /= sqrt(y_coord[0]* y_coord[0] + y_coord[1]*y_coord[1] + y_coord[2]*y_coord[2]);
    // Z
    z_coord = x_coord.cross(y_coord);
    z_coord /= sqrt(z_coord[0]*z_coord[0] + z_coord[1]*z_coord[1] +
                    z_coord[2] * z_coord[2]);
    dynacore::Matrix R(3, 3);
    R << x_coord[0], y_coord[0], z_coord[0],
         x_coord[1], y_coord[1], z_coord[1],
         x_coord[2], y_coord[2], z_coord[2];


    // Update Body Quaternion
    Eigen::Matrix3d R_mat = R;
    dynacore::Quaternion quat(R_mat);
    body_quat_ = quat;

    return R.transpose();
}

void MoCapManager::_print_message(const mercury_message & msg){
  for(int i(0); i < NUM_MARKERS; ++i){
    printf("%d th LED data (cond, x, y, z): %d, (%f, %f, %f) \n", i,
           msg.visible[i],
           msg.data[3*i],
           msg.data[3*i+1],
           msg.data[3*i+2]);
    if(i == (NUM_MARKERS - 1)) {
      // printf("size: %lu\n", sizeof(mercury_message));
      printf("\n");
    }
  }
}

void MoCapManager::_UpdateLEDPosData(const mercury_message & msg){
  dynacore::Vect3 pos;

  int led_idx = mercury_link::LED_BODY_0;

  for(int i(0); i<NUM_MARKERS; ++i){
    robot_sys_->getPos(mercury_link::LED_BODY_0 + i, pos);

    for (int j(0); j<3; ++j)  led_kin_data_[3*i + j] = pos[j];
  }

  int led_number(0);
  for(int i(0); i<3*NUM_MARKERS; ++i){
    if (msg.visible[led_number] > 0) {
      led_pos_data_[i] = msg.data[i] * 0.001;
    }

    if (i%3 == 2) {
      ++led_number;
    }
  }
}
