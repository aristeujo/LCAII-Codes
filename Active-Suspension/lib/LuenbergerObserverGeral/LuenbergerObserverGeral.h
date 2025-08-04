#ifndef LUENBERGER_OBSERVER_GERAL_H
#define LUENBERGER_OBSERVER_GERAL_H

#include <BasicLinearAlgebra.h>
using namespace BLA;

class LuenbergerObserverGeral{

  private:

    static const int systemOrderGeral = 5;

    Matrix<systemOrderGeral, systemOrderGeral> A;
    Matrix<systemOrderGeral,1> B;
    Matrix<1, systemOrderGeral> C; 
    
    Matrix<systemOrderGeral, 1> x_hat;
    Matrix<systemOrderGeral, 1> L;
  
  public:

    LuenbergerObserverGeral(Matrix<systemOrderGeral, systemOrderGeral> A, Matrix<systemOrderGeral,1> B, Matrix<1, systemOrderGeral> C);
    ~LuenbergerObserverGeral();

    void setObserverGain(Matrix<systemOrderGeral, 1> L) { this->L = L; }
    Matrix<systemOrderGeral, 1> estimate(float input, float output);
};

#endif
