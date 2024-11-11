#include "kalmanFilter.h"

static const int systemOrder = 2;

KalmanFilter::KalmanFilter(Matrix<systemOrder, systemOrder> A, Matrix<systemOrder,1> B, Matrix<1, systemOrder> C, float qValue1, float qValue2, float rValue){
  
  this->A = A;
  this->B = B;
  this->C = C;
  
  this->Q = {
    qValue1, 0,
    0, qValue2
  };
  this->R = {rValue};
};

KalmanFilter::~KalmanFilter(){};

Matrix<systemOrder, 1> KalmanFilter::kalman(float input, float output){

  //a priori
  this->x_priori = this->A * this->x_hat + this->B * input;
  this->P_priori = this->A * this->P_hat * ~this->A + this->U * this->Q * ~this->U;
  
  //kalman gain
  this->K_kalman = this->P_priori * ~this->C * Inverse(this->C * this->P_priori * ~this->C + this->R);

  //a posteriori
  this->x_hat = this->x_priori + this->K_kalman * (output - (this->C * this->x_priori)(0,0));
  this->P_hat = this->P_priori - this->K_kalman * this->C * this->P_priori;
  
  return this->x_hat;
};

float KalmanFilter::getPTrace(){

  float trace = P_hat(0) + P_hat(2);

  return trace;
}
