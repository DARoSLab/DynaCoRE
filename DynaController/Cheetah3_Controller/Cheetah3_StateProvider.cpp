#include "Cheetah3_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "Cheetah3_DynaCtrl_Definition.h"

Cheetah3_StateProvider* Cheetah3_StateProvider::getStateProvider(){
    static Cheetah3_StateProvider state_provider_;
    return &state_provider_;
}

Cheetah3_StateProvider::Cheetah3_StateProvider():
                                Q_(cheetah3::num_q),
                                Qdot_(cheetah3::num_qdot),
                                b_fr_contact_(0),
                                b_fl_contact_(0),
                                b_hr_contact_(0),
                                b_hl_contact_(0)
{
  Q_.setZero();
  Qdot_.setZero();
  global_pos_local_.setZero();

  des_location_.setZero();

  DataManager* data_manager = DataManager::GetDataManager();

  data_manager->RegisterData(&curr_time_, DOUBLE, "time");
  data_manager->RegisterData(&Q_, DYN_VEC, "config", cheetah3::num_q);
  data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", cheetah3::num_qdot);
  data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

  data_manager->RegisterData(&b_fr_contact_, INT, "fr_contact", 1);
  data_manager->RegisterData(&b_fl_contact_, INT, "fl_contact", 1);
  data_manager->RegisterData(&b_hr_contact_, INT, "hr_contact", 1);
  data_manager->RegisterData(&b_hl_contact_, INT, "hl_contact", 1);
}

