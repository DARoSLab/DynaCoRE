#include "CoMCtrlTest.hpp"

#include <Mercury_Controller/CtrlSet/JPosTargetCtrl.hpp>
#include <Mercury_Controller/CtrlSet/ContactTransCoMCtrl.hpp>
#include <Mercury_Controller/CtrlSet/CoMzRxRyRzCtrl.hpp>
#include <ParamHandler/ParamHandler.hpp>
#include <Mercury/Mercury_Model.hpp>
#include <Mercury/Mercury_Definition.h>

CoMCtrlTest::CoMCtrlTest(RobotSystem* robot):Test(robot){
  phase_ = CoMCtrlPhase::initial_jpos;
  state_list_.clear();

  jpos_ctrl_ = new JPosTargetCtrl(robot);
  com_up_ctrl_ = new ContactTransCoMCtrl(robot);
  com_fix_ctrl_ = new CoMzRxRyRzCtrl(robot);
  com_shake_ctrl_ = new CoMzRxRyRzCtrl(robot);

  state_list_.push_back(jpos_ctrl_);
  state_list_.push_back(com_up_ctrl_);
  state_list_.push_back(com_fix_ctrl_);
  state_list_.push_back(com_shake_ctrl_);
  _SettingParameter();

  printf("[CoM Control Test] Constructed\n");
}

CoMCtrlTest::~CoMCtrlTest(){
  for(int i(0); i<state_list_.size(); ++i){
    delete state_list_[i];
  }
}

void CoMCtrlTest::TestInitialization(){
  // Yaml file name
  jpos_ctrl_->CtrlInitialization("CTRL_jpos_initialization");
  com_up_ctrl_->CtrlInitialization("CTRL_move_to_target_height");
  com_fix_ctrl_->CtrlInitialization("CTRL_fix_des_pos");
  com_shake_ctrl_->CtrlInitialization("CTRL_com_shake_ctrl");
}

int CoMCtrlTest::_NextPhase(const int & phase){
  int next_phase = phase + 1;
  printf("next phase: %i\n", next_phase);
  if (next_phase == CoMCtrlPhase::NUM_CoMCtrl_PHASE) {
    return CoMCtrlPhase::stay_up;
  }
  else return next_phase;
}


void CoMCtrlTest::_SettingParameter(){
  ParamHandler handler(MercuryConfigPath"TEST_com_ctrl.yaml");

  double tmp;
  std::vector<double> tmp_vec;

  handler.getVector("initial_jpos", tmp_vec);
  ((JPosTargetCtrl*)jpos_ctrl_)->setTargetPosition(tmp_vec);

  // CoM Height
  handler.getValue("com_height", tmp);
  ((ContactTransCoMCtrl*)com_up_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)com_fix_ctrl_)->setStanceHeight(tmp);
  ((CoMzRxRyRzCtrl*)com_shake_ctrl_)->setStanceHeight(tmp);

  //// Timing Setup
  handler.getValue("jpos_initialization_time", tmp);
  ((JPosTargetCtrl*)jpos_ctrl_)->setMovingTime(tmp);
  handler.getValue("com_lifting_time", tmp);
  ((ContactTransCoMCtrl*)com_up_ctrl_)->setStanceTime(tmp);

  // Stance Time
  handler.getValue("com_stay_time", tmp);
  ((CoMzRxRyRzCtrl*)com_fix_ctrl_)->setStanceTime(tmp);
  handler.getValue("com_ctrl_time", tmp);
  ((CoMzRxRyRzCtrl*)com_shake_ctrl_)->setStanceTime(tmp);

  // Motion Setup
  handler.getVector("amp", tmp_vec);
  ((CoMzRxRyRzCtrl*)com_shake_ctrl_)->setAmp(tmp_vec);
  handler.getVector("frequency", tmp_vec);
  ((CoMzRxRyRzCtrl*)com_shake_ctrl_)->setFrequency(tmp_vec);
  handler.getVector("phase", tmp_vec);
  ((CoMzRxRyRzCtrl*)com_shake_ctrl_)->setPhase(tmp_vec);
  
}
