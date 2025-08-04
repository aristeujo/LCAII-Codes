#include <Arduino.h>
#include <MPU6050_light.h>
#include <BasicLinearAlgebra.h>
// #include <TinyMPU6050.h>
// #include <filters.h>

// Constants related imports
#include <pins.h>
#include <system.h>
#include <geralSystem.h>

// Custom Librarys imports
#include <ThreePhaseEncoder.h>
// #include <CustomMPU6050.h>
#include <DamperSensor.h>
#include <UnsprungMassSensor.h>
#include <CustomServo.h>
#include <prbs.h>
#include <KalmanFilter.h>
#include <KalmanFilterGeral.h>
#include <ControllerGeral.h>
#include <LuenbergerObserver.h>
#include <LuenbergerObserverGeral.h>

using namespace BLA;

// *** TASKS RELATED CONSTANTS ***
#define USE_MPU6050
#define USE_ENCODERS
#define PRINT_SENSOR_DATA
#define UPDATE_SENSORS
#define ENABLE_SERVO_CONTROLLER
// #define TEST_SERVO
// #define ENABLE_GERAL_CONTROLLER
// #define USE_MANUAL_SETPOINT
// #define ENABLE_OVERRIDE_CONTROLLER
// #define MANUAL_PWM
// #define IDENTIFY_MOTOR
#define IDENTIFY_WITH_MOTOR_CONTROLLER
// #define USE_LUENBERGER_OBSERVER
#define USE_DIF_EQUATIONS 

#define SENSOR_UPDATE_TIME 14 // ms
#define SAMPLING_TIME_SERVO 1 // us // ms - for the controller that works but is slow
#define SAMPLING_TIME 19 // ms
#define PRBS_HOLDING_FACTOR_MOTOR 50
#define PRBS_HOLDING_FACTOR 30 // 80 // 7 para identificaçao geral usada
#define PRBS_PERIOD 12000 // <- update this value
#define PLANETARY_REDUCTION 26.9
#define PWM_DIVISION_FACTOR 1.255 // 1.2629629 // 1.27875

// LOGIC RELATED VARIABLES
unsigned long int k = 0;
float controllerEnabled = 1;

#ifdef USE_ENCODERS
ThreePhaseEncoder encoderActuator(PIN_ACTUATOR_ENCODER_A, PIN_ACTUATOR_ENCODER_B, PIN_ACTUATOR_ENCODER_Z);
ThreePhaseEncoder encoderArm(PIN_ARM_ENCODER_A, PIN_ARM_ENCODER_B, PIN_ARM_ENCODER_Z);
DamperSensor damperSensor(&encoderArm, &encoderActuator);
UnsprungMassSensor unsprungMassSensor(&encoderArm);
#endif

#ifdef USE_MPU6050
MPU6050 sprungMassMpu(Wire);
#endif

CustomServo singleLinkServo(PIN_ENABLE_SINGLE_LINK, PIN_INT1_SINGLE_LINK_PWM, PIN_INT2_SINGLE_LINK_PWM, 0, 1, 30000, 10);

ControllerGeral geralController;

#ifndef USE_LUENBERGER_OBSERVER
KalmanFilter kalmanFilter(sys.A, sys.B, sys.C, 1e-5, 1e-6); // 1.8 e 1
KalmanFilterGeral kalmanFilterGeral(sysGeral.A, sysGeral.B, sysGeral.C, 1.5e-5, 2.1e-5);
#else
LuenbergerObserver luenbergerObserver(sys.A, sys.B, sys.C);
LuenbergerObserverGeral luenbergerObserverGeral(sysGeral.A, sysGeral.B, sysGeral.C);

#endif

Matrix<systemOrder, 1> servoObserverGains;
Matrix<systemOrderGeral, 1> geralObserverGains;

Matrix<systemOrder+1,1> singleLinkGains;
Matrix<1, systemOrderGeral> geralControllerGains;

Matrix<systemOrder, 1> statesServo;
Matrix<systemOrderGeral, 1> states;

// IIR::ORDER  order  = IIR::ORDER::OD3;
// Filter lowPassFilter(20, 19*1e-3, order);

// Identification related variables
#if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
uint16_t prbsStartingValue = 0xF6CDu;
// uint16_t prbsStartingValue = 0xBDF1u;
uint16_t prbsSequence[PRBS_PERIOD+1];
uint16_t prbsBinarySequency[PRBS_PERIOD+1];
int prbsCounter = 0;
int lastCounterToPwm = prbsCounter;
uint16_t prbsMinVal = 0;
uint16_t prbsMaxVal = 0;
#endif

// Encoders interrupt functions
#ifdef USE_ENCODERS
void IRAM_ATTR encoderActuatorPhaseA() { encoderActuator.phase_A_Interrupt(); }
void IRAM_ATTR encoderActuatorPhaseB() { encoderActuator.phase_B_Interrupt(); }
void IRAM_ATTR encoderActuatorPhaseZ() { encoderActuator.phase_Z_Interrupt(); }
void IRAM_ATTR encoderArmPhaseA() { encoderArm.phase_A_Interrupt(); }
void IRAM_ATTR encoderArmPhaseB() { encoderArm.phase_B_Interrupt(); }
void IRAM_ATTR encoderArmPhaseZ() { encoderArm.phase_Z_Interrupt(); }
#endif

float eServo[systemOrder+2];
float uServo[systemOrder+2];
float yServo[systemOrder+2];
float rServo[systemOrder+2];

float e[systemOrderGeral+2];
float u[systemOrderGeral+2];
float y[systemOrderGeral+2];
float r[systemOrderGeral+2];

float difEqControl(float currentState) {
  /*
    e -> é um vetor com tamanho 2;
    u -> é um vetor com tamanho 2;

    CUIDADO: EXCEDENDO A DIMENSÃO VAI CRASHAR O CODIGO;
  */
  // float y = currentState(0, 0);
  // float r = singleLinkServo.getTarget();

  for (int i = systemOrderGeral+2-1; i > 0; i--) {
    e[i] = e[i - 1];
    u[i] = u[i - 1];
    y[i] = y[i - 1];
    r[i] = r[i - 1];
  }

  // Cálculo do erro
  y[0] = currentState;
  r[0] = 0;
  e[0] = 0 - currentState; // Usando somente o primeiro estado

  // Bruno e Iúna
  u[0] = 11.11*e[5] -23.33*e[4] +16.67*e[3] -5.5*e[2] + 0.77*e[1] -0.033*e[0] -3.333*u[4] +4.27*u[3] + 4.44*u[2] -3.78*u[1];
  
  // Ricardo
  // u[0]=0.8057*e[0] -3.276*e[1] +4.983*e[2] - 3.361*e[3] +0.8486*e[4] + 5.006*u[1] -10.05*u[2] +10.14*u[3] - 5.134*u[4] +1.046*u[5];


  u[0] = geralController.saturate(u[0]);

  return u[0];
}

float difEqControlServo(float y) {
  /*
    e -> é um vetor com tamanho systemOrder;
    u -> é um vetor com tamanho systemOrder;

    CUIDADO: EXCEDENDO A DIMENSÃO VAI CRASHAR O CODIGO;
  */

  for (int i = systemOrder+2-1; i > 0; i--) {
    eServo[i] = eServo[i - 1];
    uServo[i] = uServo[i - 1];
    yServo[i] = yServo[i - 1];
    rServo[i] = rServo[i - 1];
  }

  // Cálculo do erro
  yServo[0] = y;
  rServo[0] = singleLinkServo.getTarget();
  eServo[0] = rServo[0] - yServo[0]; // Usando somente o primeiro estado

  // if(eServo[0] > 2000)
  //   eServo[0] = 2000;
  // else if(eServo[0] < -2000)
  //   eServo[0] = -2000;

  // Cálculo do sinal de controle <----- ALTERAR A LEI DE CONTROLE AQUI
  // uServo[0] = 0;
  // uServo[0] = 0.891*uServo[2] + 0.995*uServo[1] + 2.29*yServo[2] - 2.474*yServo[1] + 0.1039*rServo[2];
  // uServo[0] = -0.891*uServo[2] + 2.777*uServo[1] + 2.4979*yServo[2] - 2.474*yServo[1] - 0.1039*rServo[2];

  // Guilherme e Ricardo
  // uServo[0] = -0.3491*eServo[0] + 0.681*eServo[1]-0.3318*eServo[2]+2.992*uServo[1] - 2.846 * uServo[2] + 0.9244 * uServo[3];
  // uServo[0] = -0.005403*eServo[0] + 0.0002328*eServo[1]-0.00508* eServo[2] + 2.992*uServo[1] - 2.846 * uServo[2] + 0.9244 * uServo[3];
  // uServo[0] = 0.1591*eServo[0]+0.3193*eServo[1]-0.1602*eServo[2]+2.981*uServo[1]-2.963*uServo[2]+0.9816*uServo[3];
  // uServo[0] = 0.0003607*eServo[0] + 0.00000592*eServo[1]-0.0003603*eServo[2]+2.981*uServo[1]-2.963*uServo[2]+0.9816*u[3];
  // uServo[0] = -1*(-1844*eServo[0] - 193500*eServo[1]-38.6*uServo[1]+1056*uServo[2]);
  // uServo[0] = -36.23*eServo[0] - 5695*eServo[1] + 30070*eServo[2] - 38.6*uServo[1]-1056*uServo[2];
  // uServo[0] = -93.22*eServo[0] - 7894*eServo[1] - 30070*eServo[2] - 38.6*uServo[1] - 1056*uServo[2];
  // uServo[0]= -1.19*eServo[0] + 1.126*eServo[1] + 1.956*uServo[1] - 0.9592*u[2];

  // Bruno e Iúna - u positivo
  // uServo[0] = 194331.984*eServo[0] - 844418.74*eServo[1] +1214574.9*eServo[2] +578369*eServo[3] -14.04*uServo[1] + 15.04*uServo[2];

  //Lucas e Schaidson
  // uServo[0] = 2.866*uServo[1] - 2.777*uServo[2] + 0.891 *uServo[3] - 0.1464*eServo[1]      - 0.000772*eServo[2] + 0.1378*eServo[3];
  // uServo[0] = 2.982*uServo[1] - 2.965*uServo[2] + 0.9828*uServo[3] + 0.0005091*eServo[1] - (1.074e-5)*eServo[2] - 0.00051*eServo[3];
  // uServo[0] = 2.989*uServo[1] - 2.979*uServo[2] + 0.9897*uServo[3] + 0.0003113*eServo[1] - (2.332e-6)*eServo[2] - 0.0003109*eServo[3];
  // uServo[0] = 2.999*uServo[1] - 2.999*uServo[2] + 0.9995*uServo[3] + 0.0001239*eServo[1] - (1.213e-7)*eServo[2] - 0.000124*eServo[3];
  // uServo[0] = 0.989*uServo[1] + 0.0103*uServo[2] + 0.002893*rServo[2] + 0.2147*yServo[1] - 0.218193*yServo[2];
  uServo[0] = 2.989*uServo[1] - 1.9897*uServo[2] - 0.002893*rServo[0] + 0.2147*yServo[1] - 0.212407*yServo[2];

  // Serial.println(uServo[0]);
  uServo[0] = singleLinkServo.saturate(uServo[0]);
  return -uServo[0]; // check for signal - negative
}

template< typename T> 
T map( const T x, T in_min, T in_max, T out_min, T out_max ) 
{ 
  if(x < in_min) return out_min;
  if(x > in_max) return out_max;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

void estimateServoStates(float input, float output) {
  #ifndef USE_LUENBERGER_OBSERVER
  statesServo = kalmanFilter.estimate(input, output);
  #else
  statesServo = luenbergerObserver.estimate(input, output);
  #endif
}

void estimateStates(float input, float output) {
  #ifndef USE_LUENBERGER_OBSERVER
  states = kalmanFilterGeral.estimate(input, output);
  #else
  states = luenbergerObserverGeral.estimate(input, output);
  #endif
}

void printSensorsData(void* parameters) {
  for(;;){ // infinite loop

    #if (defined(TEST_SERVO) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)) && !defined(ENABLE_GERAL_CONTROLLER)
    // u, y, x1, x2
    // Servo control data
    Serial.printf("%f,%f,%f,%f\n",
      singleLinkServo.getPwmOutput(),
      singleLinkServo.getTarget(),
      encoderActuator.getPulsesDegresWithCorrection()
      // statesServo(0, 0),
      // statesServo(1, 0)
    );

    vTaskDelay(1 / portTICK_PERIOD_MS);
    #endif
    
    #ifdef ENABLE_GERAL_CONTROLLER
    // flag, u, u_real, y
    Serial.printf("%d,%f,%f,%f\n",
      controllerEnabled > 0? 100:0,
      singleLinkServo.getTarget(),
      encoderActuator.getPulsesDegresWithCorrection(),
      sprungMassMpu.getAccX()*9.81
    );
    vTaskDelay(SAMPLING_TIME / portTICK_PERIOD_MS);
    #endif

    #if defined(USE_DIF_EQUATIONS) && defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
    vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
    #endif

    #if defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
    vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
    #endif
    #if !defined(ENABLE_GENERAL_CONTROLLER) && !defined(IDENTIFY_MOTOR)
    // vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
    #endif
  }
}

void updateSensorsData(void* parameters) {
  for(;;){ // infinite loop
    #ifdef USE_MPU6050
    sprungMassMpu.update();
    #endif
    #ifdef USE_ENCODERS
    encoderActuator.updateSpeed();
    encoderArm.updateSpeed();
    // damperSensor.update();
    // unsprungMassSensor.update();
    #endif

    vTaskDelay(SENSOR_UPDATE_TIME / portTICK_PERIOD_MS);
  }
}

void controlServo(void* parameters) {
  for(;;){ // infinite loop
    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&myMutex);

    #ifndef USE_DIF_EQUATIONS
    estimateServoStates(
      singleLinkServo.getPwmOutput(),
      encoderActuator.getPulsesDegresWithCorrection()
    );
    singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), statesServo);
    #else
    statesServo(0, 0) = encoderActuator.getPulsesDegresWithCorrection();
    float pwm = difEqControlServo(statesServo(0, 0));
    singleLinkServo.apply(statesServo(0, 0), pwm);
    #endif

    taskEXIT_CRITICAL(&myMutex);

    // Serial.println(pwm);
    vTaskDelay(SAMPLING_TIME_SERVO / portTICK_PERIOD_MS);
  }
}

void controlAll(void* parameters) {
  for(;;){ // infinite loop

    if(controllerEnabled > 0) {
      if (millis() > 60000)
        controllerEnabled = 0;

      singleLinkServo.enableServo();
      
      portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
      taskENTER_CRITICAL(&myMutex);

      #ifndef USE_DIF_EQUATIONS
      estimateStates(singleLinkServo.getTarget()-90, sprungMassMpu.getAccX() * 9.81);
      states(0, 0) = sprungMassMpu.getAccX() * 9.81; // check if this line back is ok
      float targetPos = geralController.controlLaw(states);
      #else
      float targetPos = difEqControl(sprungMassMpu.getAccX() * 9.81);
      // Serial.println(targetPos);
      #endif
      singleLinkServo.setTarget(targetPos+90);
      taskEXIT_CRITICAL(&myMutex);
    }
    else {
      singleLinkServo.setTarget(90);
      // singleLinkServo.disableServo();
    }

    vTaskDelay(SAMPLING_TIME / portTICK_PERIOD_MS);
  }
}

#if defined(RUN_IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
void runIdentification(void* parameters) {
  for(;;){ // infinite loop
    #ifdef IDENTIFY_MOTOR
    float pwmToApply = 0;

    if(prbsCounter <= PRBS_PERIOD) {
      pwmToApply = map(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, -singleLinkServo.getMaxPWM(), singleLinkServo.getMaxPWM());
      // pwmToApply = map(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, -750, 750);

      // if(prbsBinarySequency[prbsCounter] == 0)
      //   pwmToApply = 600;
      // else
      //   pwmToApply = -600;
      
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }
    else if (prbsCounter == PRBS_PERIOD+1) {
      Serial.println("End Of Motor Identification");
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }

    // pwmToApply = 300;
    singleLinkServo.applyPwmToMotor(pwmToApply);

    vTaskDelay(SAMPLING_TIME_SERVO * PRBS_HOLDING_FACTOR_MOTOR / portTICK_PERIOD_MS);
    // delayMicroseconds(SAMPLING_TIME_SERVO * PRBS_HOLDING_FACTOR_MOTOR);
    #endif

    #ifdef IDENTIFY_WITH_MOTOR_CONTROLLER
    float angleToApply = 0;

    if(prbsCounter <= PRBS_PERIOD) {
      // Serial.println("pwm to apply: " + String(prbsSequence[prbsCounter]));
      angleToApply = map<float>(prbsSequence[prbsCounter], prbsMinVal, prbsMaxVal, 90-65, 90+65);

      // if(prbsBinarySequency[prbsCounter] == 0)
      //   angleToApply = 80;
      // else
      //   angleToApply = 100;
      
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }
    else if (prbsCounter == PRBS_PERIOD+1) {
      Serial.println("End Of Motor Identification with controller");
      angleToApply = 0;
      lastCounterToPwm = prbsCounter;
      prbsCounter++;
    }

    // angleToApply = 111;
    singleLinkServo.setTarget(angleToApply);

    vTaskDelay(SAMPLING_TIME * PRBS_HOLDING_FACTOR / portTICK_PERIOD_MS);
    #endif
  }
}
#endif

#ifdef USE_ENCODERS
void encodersSetup() {
  // Encoders max value setup
  // taking into account the planetary gear box reduction wich is 29 + 1 (for margin)
  encoderActuator.setMaxAngle(720*600);
  encoderActuator.setCorrectionFactor(PLANETARY_REDUCTION);

  // Encoders interrupt setup
  pinMode(encoderActuator.pA(), INPUT_PULLDOWN);
  pinMode(encoderActuator.pB(), INPUT_PULLDOWN);
  pinMode(encoderActuator.pZ(), INPUT_PULLDOWN);
  pinMode(encoderArm.pA(), INPUT_PULLDOWN);
  pinMode(encoderArm.pB(), INPUT_PULLDOWN);
  pinMode(encoderArm.pZ(), INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pA()), encoderActuatorPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pB()), encoderActuatorPhaseB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderActuator.pZ()), encoderActuatorPhaseZ, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pA()), encoderArmPhaseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pB()), encoderArmPhaseB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderArm.pZ()), encoderArmPhaseZ, RISING);
}
#endif

void setup() {
  // Serial printer
  // Serial.begin(1000000);

  pinMode(PIN_TRIGGER, OUTPUT); // Sets the trigPin as an Output
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(2);

  servoObserverGains = {
    3.03446789302781,
    -2.95501643410786
  };
  geralObserverGains = {0, 0, 0, 0, 0};

  // MODELO MOLA FORTE - h2 - lmis-separadas
  singleLinkGains = { // - 217.465227308753
    -12980.832502934,
    -10945.7475046691,
    148.194277791022
  };

  // teste controlador h2 - alpha = .95 - lmis isoladas
  geralControllerGains = {
    -1.34216705215132,
    -2.2099873300152,
    1.06310444625557,
    0.995545083167062,
    0.92790872284339
  };

  #ifdef USE_LUENBERGER_OBSERVER
  luenbergerObserver.setObserverGain(servoObserverGains);
  luenbergerObserverGeral.setObserverGain(geralObserverGains);
  #endif

  singleLinkServo.setMaxPwm(singleLinkServo.getMaxPWM() / PWM_DIVISION_FACTOR);
  singleLinkServo.setTrackingGains(singleLinkGains);
  singleLinkServo.setTarget(90);

  geralController.setRegulationGains(geralControllerGains);
  geralController.setSaturationLimits(-60, 60);

  #ifdef USE_ENCODERS
  encodersSetup();
  #endif

  #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
  Serial.println("Motor Identification RUN");
  prbs(prbsStartingValue, PRBS_PERIOD, prbsSequence, prbsBinarySequency);
  prbsMaxVal = prbsSequence[0];
  prbsMinVal = prbsSequence[0];

  for (int i = 0; i < (sizeof(prbsSequence) / sizeof(prbsSequence[0])); i++) {
      if (prbsSequence[i] > prbsMaxVal) {
         prbsMaxVal = prbsSequence[i];
      }
      if (prbsSequence[i] < prbsMinVal) {
         prbsMinVal = prbsSequence[i];
      }
   }
  #endif

  // MPU6050 setup
  #ifdef USE_MPU6050
  Wire.begin();
  byte status = sprungMassMpu.begin(1, 0);
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  sprungMassMpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  #endif

  // Zero routine for the second controller
  #ifdef ENABLE_SERVO_CONTROLLER
  singleLinkServo.setTarget(90);
  int counter = 0;

  Serial.begin(1000000);
  Serial.println("zeroing servo in 90 deg");
  while(true) {
    #ifndef USE_DIF_EQUATIONS
    estimateServoStates(
      singleLinkServo.getPwmOutput(),
      encoderActuator.getPulsesDegresWithCorrection()
    );

    singleLinkServo.update(encoderActuator.getPulsesDegresWithCorrection(), statesServo);
    #else
    statesServo(0, 0) = encoderActuator.getPulsesDegresWithCorrection();
    float pwm = difEqControlServo(statesServo(0, 0));
    singleLinkServo.apply(statesServo(0, 0), pwm);
    Serial.println(pwm);
    // Serial.println(encoderActuator.getPulsesDegresWithCorrection());
    #endif

    if(abs(singleLinkServo.getTarget() - encoderActuator.getPulsesDegresWithCorrection()) / abs(singleLinkServo.getTarget() + 0.0001) * 100 <= 2)
      counter++;

    if(counter >= 1000)
      break;

    delay(SAMPLING_TIME_SERVO);
  }
  #endif

  // Tasks Creation
  #ifdef PRINT_SENSOR_DATA
  xTaskCreatePinnedToCore(
    printSensorsData,
    "Print",
    5000,
    NULL,
    15,
    NULL,
    1
  );
  #endif

  #ifdef UPDATE_SENSORS
  xTaskCreatePinnedToCore(
    updateSensorsData,
    "Update all the sensors data",
    5000,
    NULL,
    5,
    NULL,
    1
  );
  #endif

  #if defined(IDENTIFY_MOTOR) || defined(IDENTIFY_WITH_MOTOR_CONTROLLER)
  xTaskCreatePinnedToCore(
    runIdentification,
    "RunIdent",
    3000,
    NULL,
    8,
    NULL,
    0
  );
  #endif

  #if !defined(IDENTIFY_MOTOR) && defined(ENABLE_SERVO_CONTROLLER)
  xTaskCreatePinnedToCore(
    controlServo,
    "UpdateServo",
    3000,
    NULL,
    15,
    NULL,
    0
  );
  #endif

  #if defined(ENABLE_GERAL_CONTROLLER) // && !defined(IDENTIFY_MOTOR) && !defined(IDENTIFY_WITH_MOTOR_CONTROLLER) && 
  xTaskCreatePinnedToCore(
    controlAll,
    "Update geral controller",
    6000,
    NULL,
    15,
    NULL,
    0
  );
  #endif

  #ifdef USE_ENCODERS
  damperSensor.updateZero();
  #endif

  encoderArm.updateZero();

  // Serial.begin(1000000);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
}

void loop() {

  #ifdef USE_MANUAL_SETPOINT
  if (Serial.available() > 0) {
    Serial.setTimeout(5);
    String floatString = Serial.readString();
    Serial.flush();
    #ifndef MANUAL_PWM
    float targetPosition = floatString.toFloat();

    #ifdef ENABLE_OVERRIDE_CONTROLLER
    controllerEnabled = targetPosition;
    #endif
    #ifndef ENABLE_OVERRIDE_CONTROLLER
    singleLinkServo.setTarget(targetPosition);
    #endif

    #endif

    #ifdef MANUAL_PWM
    float targetPwm = floatString.toFloat();
    // singleLinkServo.applyPwmToMotor(targetPwm);
    #endif
  }
  #endif

  // sinosoidal input for servo controller test
  #if defined(TEST_SERVO) && !defined(ENABLE_GERAL_CONTROLLER)

  singleLinkServo.setTarget(90 + 60 * sin(2*PI*k*1/1000));
  k++;
    
  vTaskDelay(1 / portTICK_PERIOD_MS);
  #endif
}