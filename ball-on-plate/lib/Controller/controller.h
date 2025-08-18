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
  ControllerType controllerType;
  static const int systemOrder = 2;
  std::function<float(Matrix<systemOrder, 1>, float*, float*)> diffEquationsControlFunc;
  Matrix<1, systemOrder> K; 

  float e[systemOrder+1] = {}; // Coeficientes na seguinte ordem e[n] = {a*e[k-1], b*e[k-2], ..., n*e[k-n]} com n = ordem
  float u[systemOrder+1] = {};  // Coeficientes na seguinte ordem u[m] = {x*u[k-1], y*u[k-2], ..., z*e[k-m]} com m = ordem
public:  
  Controller(Matrix<1, systemOrder> gains);
  Controller(std::function<float(Matrix<systemOrder, 1>, float*, float*)> func);
  ~Controller();
  float controlLaw(Matrix<systemOrder, 1> currentState);
};
#endif
