
//BY--ZJ 2020.10.12
#ifndef FSM_STATE_DOWN_H
#define FSM_STATE_DOWN_H

#include "FSM_State.h"
#include "Utilities/BSplineBasic.h"
#include "Controllers/LegController.h"
template <typename T>  
class FSM_State_Down : public FSM_State<T> {
 public:
  FSM_State_Down(ControlFSMData<T>* _controlFSMData);

  
  void onEnter();//进入模式

  
  void run();//执行


  FSM_StateName checkTransition();


  TransitionData<T> transition();

 
  void onExit();

  TransitionData<T> testTransition();

 private:

  int iter = 0;
  bool flag_down = true;
  BS_Basic<T, 12, 3, 1, 2, 2> _jpos_trj;
  LegController<T>* ctrl;

 
  Vec3<T> target_jpos[4],initial_jpos[4];
  Vec3<T> f_ff;
  Vec3<T> zero_vec3;
  
 // Vec3<T> &ini = initial_jpos;
 // Vec3<T> &fin = target_jpos;
  const int max_time = 5000;
  std::vector< Vec3<T> > _ini_foot_pos;  //切换时的足端位置
};














#endif 
