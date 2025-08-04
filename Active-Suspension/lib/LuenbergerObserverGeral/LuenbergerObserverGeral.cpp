#include "LuenbergerObserverGeral.h"

LuenbergerObserverGeral::LuenbergerObserverGeral(Matrix<systemOrderGeral, systemOrderGeral> A, Matrix<systemOrderGeral,1> B, Matrix<1, systemOrderGeral> C){

  this->A = A;
  this->B = B;
  this->C = C;
  this->L = L;
  
};

LuenbergerObserverGeral::~LuenbergerObserverGeral(){};

Matrix<LuenbergerObserverGeral::systemOrderGeral, 1> LuenbergerObserverGeral::estimate(float input, float output){
  x_hat = A * x_hat + B*input + L * (output - (C * x_hat)(0,0));
  
  return x_hat;
}