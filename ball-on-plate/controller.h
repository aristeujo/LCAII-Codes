#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <BasicLinearAlgebra.h>
#include <functional>
using namespace BLA;

void saturate(float* input, float lowerLimit, float upperLimit);
enum ControllerType {
  STATE_FEEDBACK,
  DIFFERENCE
};

class Controller{

private:
  std::function<float(Matrix<2, 1>, float*, float*)> diffEquationsControlFunc;
  ControllerType controllerType;
  static const int systemOrder = 2;
  Matrix<1, systemOrder> K; 

  float e[systemOrder] = {}; // Coeficientes na seguinte ordem e[n] = {a*e[k-1], b*e[k-2], ..., n*e[k-n]} com n = ordem
  float u[systemOrder] = {};  // Coeficientes na seguinte ordem u[m] = {x*u[k-1], y*u[k-2], ..., z*e[k-m]} com m = ordem
public:  
  Controller(Matrix<1, systemOrder> gains);
  Controller(std::function<float(Matrix<2, 1>, float*, float*)> diffEquationsControlFunc);
  ~Controller();
  float controlLaw(Matrix<systemOrder, 1> currentState);
};
#endif
