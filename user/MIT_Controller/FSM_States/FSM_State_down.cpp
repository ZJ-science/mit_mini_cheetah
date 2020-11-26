
//by ---zj 2020.10.12

#include "FSM_State_down.h"
#include "JPosInitializer.h"
#include<iostream>
using namespace std;

template<typename T > //notice
FSM_State_Down<T>::FSM_State_Down(ControlFSMData< T >* _controlFSMData):FSM_State<T>(_controlFSMData, FSM_StateName::Down ,"Down!"),_ini_foot_pos(4)
{
    this->checkForceFeedForward = false;
    this->checkSafeOrientation = false;
    this->checkPDesFoot = false;
    zero_vec3.setZero();
    
}

template< typename T>
FSM_StateName FSM_State_Down<T>::checkTransition()
{
    this->nextStateName = this->stateName;
    iter++;
    switch ((int)this->_data->controlParameters->control_mode)
    {
        case K_STAND_UP:
            this->nextStateName = FSM_StateName::STAND_UP;
            break;
        case K_Down:
            break;
        case K_RECOVERY_STAND:
            this->nextStateName = FSM_StateName::RECOVERY_STAND;
            break;
        case K_TC:
            cout<<"error"<<endl;
            break;
        case K_PASSIVE:
            this->nextStateName = FSM_StateName::PASSIVE;
            break;
        default:
            cout<<"Cannot transition from K_Down to"<<this->_data->controlParameters->control_mode<<endl;
            
    }
    return this->nextStateName;
}

template<typename T>
void FSM_State_Down<T>::onEnter()
{
    this->nextStateName = this->stateName;
    this->transitionData.zero();
    iter = 0;
    for(size_t i(0); i < 4; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i].q;
    }
    target_jpos[0]<<-0.6,-1.0,2.7;
    target_jpos[1]<<0.6,-1.0,2.7;
    target_jpos[2]<<-0.6,-1.0,2.7;
    target_jpos[3]<<0.6,-1.0,2.7;
    f_ff << 0.f, 0.f, -25.f;
    cout<<"Down init setup!"<<endl;
}

template<typename T> 
void FSM_State_Down<T>::onExit()
{
    cout<<"离开趴着模式\n"<<endl;
}

//
template<typename T>
void FSM_State_Down<T>::run()
{
    if(this->_data->_quadruped->_robotType == RobotType::MINI_CHEETAH)
    {
  for(size_t leg=0;leg<4 ; leg++)
  {
      float a(0.f);
      float b(1.f);

      if(iter <= max_time) {
      b = (float)iter/(float)max_time;
      a = 1.f - b;
    }
      Vec3<T> inter_pos = a*initial_jpos[leg]+b*target_jpos[leg];
      this->jointPDControl(leg,inter_pos,zero_vec3);
  }
  }
}

template<typename T> 
TransitionData<T> FSM_State_Down<T>::transition()
{
    switch (this->nextStateName)
    {
        case FSM_StateName::PASSIVE:
            this->transitionData.done =true;
            break;
        case FSM_StateName::RECOVERY_STAND:
            this->transitionData.done = true;
            break;
        case FSM_StateName::STAND_UP:
            this->transitionData.done = true;
            break;
        default:
            cout<<"down_error_zj"<<endl;
    }
    return this->transitionData;
}

template class FSM_State_Down<float>;
