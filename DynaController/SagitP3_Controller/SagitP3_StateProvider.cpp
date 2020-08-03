#include "SagitP3_StateProvider.hpp"
#include <Utils/DataManager.hpp>
#include "SagitP3_DynaCtrl_Definition.h"
#include <SagitP3/SagitP3_Model.hpp>

SagitP3_StateProvider* SagitP3_StateProvider::getStateProvider(){
    static SagitP3_StateProvider state_provider_;
    return &state_provider_;
}

SagitP3_StateProvider::SagitP3_StateProvider():
                                //stance_foot_(sagitP3_link::l_ankle),
                                stance_foot_(sagitP3_link::l_foot),
                                Q_(sagitP3::num_q),
                                Qdot_(sagitP3::num_qdot),
                                rotor_inertia_(sagitP3::num_act_joint),
                                b_rfoot_contact_(0),
                                b_lfoot_contact_(0),
                                reaction_forces_(10)
{
    rotor_inertia_.setZero();
    Q_.setZero();
    Qdot_.setZero();
    global_pos_local_.setZero();
    reaction_forces_.setZero();
    des_location_.setZero();

    rfoot_pos_.setZero();
    lfoot_pos_.setZero();
    rfoot_vel_.setZero();
    lfoot_vel_.setZero();

    DataManager* data_manager = DataManager::GetDataManager();

    data_manager->RegisterData(&curr_time_, DOUBLE, "time");
    data_manager->RegisterData(&Q_, DYN_VEC, "config", sagitP3::num_q);
    data_manager->RegisterData(&Qdot_, DYN_VEC, "qdot", sagitP3::num_qdot);
    data_manager->RegisterData(&global_pos_local_, VECT3, "global_pos_local", 3);

    data_manager->RegisterData(&b_rfoot_contact_, INT, "rfoot_contact", 1);
    data_manager->RegisterData(&b_lfoot_contact_, INT, "lfoot_contact", 1);

    data_manager->RegisterData(&reaction_forces_, DYN_VEC, "reaction_force", 10);

   data_manager->RegisterData(&rfoot_pos_, VECT3, "rfoot_pos", 3); 
   data_manager->RegisterData(&lfoot_pos_, VECT3, "lfoot_pos", 3); 
   data_manager->RegisterData(&rfoot_vel_, VECT3, "rfoot_vel", 3); 
   data_manager->RegisterData(&lfoot_vel_, VECT3, "lfoot_vel", 3); 
}

void SagitP3_StateProvider::SaveCurrentData(const RobotSystem* robot_sys){
    robot_sys->getPos(sagitP3_link::r_foot, rfoot_pos_);
    robot_sys->getPos(sagitP3_link::l_foot, lfoot_pos_);
    robot_sys->getPos(sagitP3_link::r_foot, rfoot_vel_);
    robot_sys->getPos(sagitP3_link::l_foot, lfoot_vel_);
}
