#include "touchScreen.h"
#include "servoControl.h"
#include "controller.h"
#include "kalmanFilter.h"
#include "stateSpaceMatrices.h"
#include <functional>

#define USE_STATE_FEEDBACK

float difEqControlX(Matrix<systemOrder,1> currentState, float* e, float* u);
float difEqControlY(Matrix<systemOrder,1> currentState, float* e, float* u);

std::function<float(Matrix<2, 1>, float*, float*)> difX;
std::function<float(Matrix<2, 1>, float*, float*)> difY;

ServoControl servos(23, 19);
TouchScreen ts(27, 26, 32, 33, 25);
screenCoordinates coords;
screenCoordinatesCm coordsCm;

KalmanFilter xFilter(sys.A, sys.B, sys.C, 0.01, 200, 150);
KalmanFilter yFilter(sys.A, sys.B, sys.C, 0.01, 200, 150);

Matrix<1,2> stateFeedbackGains = {0.0, 0.0};

#ifdef USE_STATE_FEEDBACK
Controller xController(stateFeedbackGains);
Controller yController(stateFeedbackGains);
#else
Controller xController(difX);
Controller yController(difY);
#endif

float angleX = 0;
float angleY = 0;

float posX = 0;
float posY = 0;

float uX = 0;
float uY = 0;

float uDegreeX = 0;
float uDegreeY = 0;

float referenceX = 0;
float referenceY = 0;

Matrix<2, 1> statesX = {0, 0};
Matrix<2, 1> statesY = {0, 0};

float difEqControlX(Matrix<2,1> currentState, float* e, float* u) {
  /*
    e -> é um vetor com tamanho 2;
    u -> é um vetor com tamanho 2;

    CUIDADO: EXCEDENDO A DIMENSÃO VAI CRASHAR O CODIGO;
  */

  for (int i = 2; i > 0; i--) {
    e[i] = e[i - 1];
    u[i] = u[i - 1];
  }

  // Cálculo do erro
  e[0] = 0 - currentState(0); // Usando somente o primeiro estado

  // Cálculo do sinal de controle <----- ALTERAR A LEI DE CONTROLE AQUI
  u[0] = 1.1329 * u[1] - 0.1329 * u[2] + 6.7469 * e[1] - 6.0454 * e[2];

  return u[0];
}

float difEqControlY(Matrix<2,1> currentState, float* e, float* u) {
  /*
    e -> é um vetor com tamanho 2;
    u -> é um vetor com tamanho 2;

    CUIDADO: EXCEDENDO A DIMENSÃO VAI CRASHAR O CODIGO;
  */

  for (int i = 2; i > 0; i--) {
    e[i] = e[i - 1];
    u[i] = u[i - 1];
  }

  // Cálculo do erro
  e[0] = 0 - currentState(0); // Usando somente o primeiro estado

  // Cálculo do sinal de controle <----- ALTERAR A LEI DE CONTROLE AQUI
  u[0] = 1.1329 * u[1] - 0.1329 * u[2] + 6.7469 * e[1] - 6.0454 * e[2];

  return u[0];
}

void setup() {
  delay(500);
  Serial.begin(1000000);
  delay(500);
  ts.setSamplingTime(35);
  servos.startPosition();
  delay(500);
}

//This is here purely for convenience. Otherwise the functions defined inside won't know about the variables.
#include "printInfo.h" 

void loop() {
  coords = ts.getCoordinates();
 
  if(ts.screenUpdated()){
    
    coordsCm = ts.getCoordinatesCm(coords.x, coords.y);
    
    posX = coordsCm.xCm * 0.01; // meter
    posY = -coordsCm.yCm * 0.01; // meter

    statesX = xFilter.kalman(uX, posX);
    statesY = yFilter.kalman(uY, posY);

    uX = xController.controlLaw(statesX);   
    uDegreeX = rad2deg(uX);
    saturate(&uDegreeX, -25, 25);
    uX = deg2rad(uDegreeX);

    uY = yController.controlLaw(statesY);   
    uDegreeY = rad2deg(uY);
    saturate(&uDegreeY, -25, 25);
    uY = deg2rad(uDegreeY);

    angleX = (uDegreeX) + servos.offset1;
    angleY = (uDegreeY) + servos.offset2;

    servos.moveServos(angleX , angleY);
    printXY();
  }
}
