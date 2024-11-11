#include "controller.h"

Controller::Controller(Matrix<1, systemOrder> gains){
  this->controllerType = ControllerType::STATE_FEEDBACK;
  this->K = gains;
};

Controller::Controller(std::function<float(Matrix<2, 1>, float*, float*)> func){
  this->controllerType = ControllerType::DIFFERENCE;
  this->diffEquationsControlFunc = diffEquationsControlFunc;
}

Controller::~Controller(){};

float Controller::controlLaw(Matrix<systemOrder,1> currentState){
  if(controllerType == ControllerType::STATE_FEEDBACK)
    return (this->K * currentState)(0);
  else
    return this->diffEquationsControlFunc(currentState, e, u);
}

void saturate(float* input, float lowerLimit, float upperLimit){
  if(*input < lowerLimit) *input = lowerLimit;
  if(*input > upperLimit) *input = upperLimit;
}
