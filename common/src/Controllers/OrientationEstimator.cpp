/*! @file OrientationEstimator.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation 通过imu获得的角度可以转换成旋转矩阵，将变量都映射到世界坐标系下
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 *  估计orientation的算法， 
 * rBody是一个变换矩阵，满足将世界坐标系的速度能映射成基躯干的速度
 */

#include "Controllers/OrientationEstimator.h"
//cheater时候的orientation估计
template <typename T>
void CheaterOrientationEstimator<T>::run() {
  this->_stateEstimatorData.result->orientation =
      this->_stateEstimatorData.cheaterState->orientation.template cast<T>();
      //通过orientation得到的旋转矩阵表示 从世界坐标变换到机体坐标的变换， 从而实现 vBody = Rbody * vWorld
  this->_stateEstimatorData.result->rBody = ori::quaternionToRotationMatrix(
      this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->omegaBody =
      this->_stateEstimatorData.cheaterState->omegaBody.template cast<T>();
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->omegaBody;
  this->_stateEstimatorData.result->rpy =
      ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.cheaterState->acceleration.template cast<T>();
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *
      this->_stateEstimatorData.result->aBody;
     
/*
  this->_stateEstimatorData.result->orientation[0] =1;// this->_stateEstimatorData.vectorNavData->quat[3]; 
  this->_stateEstimatorData.result->orientation[1] =0;// this->_stateEstimatorData.vectorNavData->quat[0];
  this->_stateEstimatorData.result->orientation[2] =0;// this->_stateEstimatorData.vectorNavData->quat[1];
  this->_stateEstimatorData.result->orientation[3] =0;// this->_stateEstimatorData.vectorNavData->quat[2];
  
  this->_stateEstimatorData.result->rBody =
			ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);
  // imu测量的陀螺 表示速度
  this->_stateEstimatorData.result->omegaBody(0) =0.0;
  this->_stateEstimatorData.result->omegaBody(1) =0.0;
  this->_stateEstimatorData.result->omegaBody(2) =0.0;
			
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->omegaBody;
      
  this->_stateEstimatorData.result->rpy = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  
    std::cout<<"CheaterOrientationEstimator : "<<this->_stateEstimatorData.result->rpy<<std::endl;
    
  this->_stateEstimatorData.result->aBody(0) =0.0;
  this->_stateEstimatorData.result->aBody(1) =0.0;
  this->_stateEstimatorData.result->aBody(2) =9.8;
      
      
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->aBody;
      */
}
//真正运行机器人时候的orientation估计
template <typename T>
void VectorNavOrientationEstimator<T>::run() {
  //imu测量的四元数表示姿态  
			  //orientation[0]: w :vectorNavData->quat[3]
			  //orientation[1]: x :vectorNavData->quat[0]
			  // …………………………………………
   this->_stateEstimatorData.result->orientation[0] = this->_stateEstimatorData.vectorNavData->quat[3]; //
   this->_stateEstimatorData.result->orientation[1] = this->_stateEstimatorData.vectorNavData->quat[0];
   this->_stateEstimatorData.result->orientation[2] = this->_stateEstimatorData.vectorNavData->quat[1];
   this->_stateEstimatorData.result->orientation[3] = this->_stateEstimatorData.vectorNavData->quat[2];
//    this->_stateEstimatorData.result->orientation[0] = this->_stateEstimatorData.vectorNavData->quat[1]; //
//    this->_stateEstimatorData.result->orientation[1] = this->_stateEstimatorData.vectorNavData->quat[2];
//    this->_stateEstimatorData.result->orientation[2] = this->_stateEstimatorData.vectorNavData->quat[3];
//    this->_stateEstimatorData.result->orientation[3] = this->_stateEstimatorData.vectorNavData->quat[0];

  
  this->_stateEstimatorData.result->rBody =
			ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);
// imu测量的陀螺 表示速度
  this->_stateEstimatorData.result->omegaBody =
			this->_stateEstimatorData.vectorNavData->gyro.template cast<T>();
  this->_stateEstimatorData.result->omegaWorld =
			this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->omegaBody;
  this->_stateEstimatorData.result->rpy = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  //imu测量的acc
  this->_stateEstimatorData.result->aBody =
      this->_stateEstimatorData.vectorNavData->accelerometer.template cast<T>();
      
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->aBody;
      
 /* static int i = 0;
  i++;
  if(i >= 15)
  {
    i = 0;
    std::cout.precision(2);
    std::cout<<"----------VectorNavOrientationEstimator-------"<<std::endl; 
    std::cout<<"-----quat:"<<std::endl<<this->_stateEstimatorData.result->orientation[0]<<"\t"
			    <<this->_stateEstimatorData.result->orientation[1]<<"\t"
			    <<this->_stateEstimatorData.result->orientation[2]<<"\t"
			    <<this->_stateEstimatorData.result->orientation[3]<<"\t"
			    <<std::endl;
    std::cout<<"--rBody--"<<std::endl<<this->_stateEstimatorData.result->rBody<<std::endl;
     std::cout.precision(2);
     for(int ii=0;ii<3;ii++)
     {
       if(fabs(this->_stateEstimatorData.result->omegaBody[ii])<0.1) 
	this->_stateEstimatorData.result->omegaBody[ii]=0;
     }
    std::cout<<"omegaBody:"<<std::endl<<this->_stateEstimatorData.result->omegaBody<<std::endl;			    
    std::cout<<"------rpy:"<<std::endl<<this->_stateEstimatorData.result->rpy<<std::endl;   
    std::cout<<"---aWorld:"<<std::endl<<this->_stateEstimatorData.result->aWorld<<std::endl;
    std::cout<<"----------------------------------------------------"<<std::endl;
  }*/
  

 
 
 //以下是作弊模式
/* 
   this->_stateEstimatorData.result->orientation[0] =1;// this->_stateEstimatorData.vectorNavData->quat[3]; 
  this->_stateEstimatorData.result->orientation[1] = 0;//this->_stateEstimatorData.vectorNavData->quat[0];
  this->_stateEstimatorData.result->orientation[2] = 0;//this->_stateEstimatorData.vectorNavData->quat[1];
  this->_stateEstimatorData.result->orientation[3] = 0;//this->_stateEstimatorData.vectorNavData->quat[2];
  
  this->_stateEstimatorData.result->rBody =
			ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);
  // imu测量的陀螺 表示速度
  this->_stateEstimatorData.result->omegaBody(0) =0.0;
  this->_stateEstimatorData.result->omegaBody(1) =0.0;
  this->_stateEstimatorData.result->omegaBody(2) =0.0;
  
  this->_stateEstimatorData.result->omegaWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->omegaBody;
      
  this->_stateEstimatorData.result->rpy = ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  //std::cout<<"VectorNavOrientationEstimator : "<<this->_stateEstimatorData.result->rpy<<std::endl;
  
  //imu测量的acc
 this->_stateEstimatorData.result->aBody(0) =0.0;
  this->_stateEstimatorData.result->aBody(1) =0.0;
  this->_stateEstimatorData.result->aBody(2) =9.8;
      
  this->_stateEstimatorData.result->aWorld =
      this->_stateEstimatorData.result->rBody.transpose() *this->_stateEstimatorData.result->aBody;
      */
}


template class CheaterOrientationEstimator<float>;
template class CheaterOrientationEstimator<double>;

template class VectorNavOrientationEstimator<float>;
template class VectorNavOrientationEstimator<double>;