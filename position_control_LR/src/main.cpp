#include <Arduino.h>
#include <stdint.h>

uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch);
void calcula_velocidade_motor();
void verifica_canal_A();
void verifica_canal_B();
void servo_controle_dif_eq();

int motorPin1 = 18;
int motorPin2 = 19;
int enablePin = 4;

#define potPin  34

const int freq = 20000;
const int pwmChannel = 2;
const int resolution = 8;

// Portas dos canais do encoder
#define ENC_A 12 // fio verde
#define ENC_B 13 // fio amarelo

// Pulsos por revolução
#define ENC_COUNT_REV_M 1368.4

// Variáveis para a contagem dos pulsos
volatile long pulsos_motor = 0;
signed long PontoAtual_enc_motor = 0;

// Variáveis do PRBS
static uint16_t stretch_counter = 0;
uint16_t bitsize = 14;
uint16_t i = random(0x01 << bitsize);
uint16_t stretch = 20;
uint16_t stretch2 = 10;

// Variável do Blink sem delay
unsigned long timer_prbs = 0;
int taxaDeAmostragem = 35;

// Sinal de controle em PWM
int sinal_controle = 0;

// Define o sentido de rotação
int prbs_rotation = 0;
int sentido = 0;

// Medições
float rpm = 0.0;
float omega = 0.0;
float angulo_motor = 0;

//// controle
int r = 0;
float u_sig = 0;
float x_i = 0;
int max_vel = 30;

int ordem = 3;   // Definir a ordem do seu controlador

float e[3] = {0, 0, 0}; // Coeficientes na seguinte ordem e[n] = {a*e[k-1], b*e[k-2], ..., n*e[k-n]} com n = ordem
float u[3] = {0, 0, 0};  // Coeficientes na seguinte ordem u[z] = {x*u[k-1], y*u[k-2], ..., z*e[k-z]} com z = ordem
float uc[3] = {0, 0, 0};

void setup() {
  Serial.begin(500000);
  pinMode(ENC_A , INPUT);
  pinMode(ENC_B , INPUT);
  pinMode(potPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), verifica_canal_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), verifica_canal_B, CHANGE);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enablePin, pwmChannel);

  Serial.println("End of Setup");
}

void loop() {
    servo_controle_dif_eq();
}

void servo_controle_dif_eq() {
  int r = map(analogRead(potPin), 0, 4095, 0, 180);

  if (millis() - timer_prbs >= taxaDeAmostragem) {
    
    for (int i = 2; i > 0; i--) {
      e[i] = e[i - 1];
      uc[i] = uc[i - 1];
    }

    // Cálculo do sinal de controle
    e[0] = r - angulo_motor*57.2958;

  //  uc[0] = 10.36 * e[0] - 10.35 * e[1] + uc[1] ;
    uc[0] = 14.073 * e[0] - 14.0561 * e[1] + uc[1] ;

    if(uc[0] > 0){
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    }
    else{
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    }

    sinal_controle = abs(uc[0]);
    sinal_controle = constrain(sinal_controle, 0, 255); // saturação do sinal de controle

    // Atualiza o comando para o motor
    ledcWrite(pwmChannel, sinal_controle);

    Serial.print(angulo_motor*57.2958); // Ângulo medido em graus
    Serial.print(",");
    Serial.println(r); // Ângulo de referência em graus

    timer_prbs = millis();
  }
}

uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch)
{
  if (stretch_counter >= stretch) {
    stretch_counter = 0;

    uint16_t mask = (1 << bits) - 1;
    uint16_t comp = (*input ^ (*input >> 1)) & 1;
    *input = ((*input >> 1) | (comp << (bits - 1))) & mask; // Bit-shifting and adding the XOR result
  } else {
    stretch_counter += 1;
  }
  return *input & 0x01;
}

void identificacao() {
  if ((millis() - timer_prbs) >= taxaDeAmostragem) {

    // Sinais PRBS
    sinal_controle = (prbs(&i, bitsize, stretch) - 0.5) * 170 + 170;
    prbs_rotation = prbs(&i, bitsize, stretch2);

    if (!prbs_rotation) {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      sentido = 1;
    }
    else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    sentido = 0;
    }

    // Atualiza o comando para o motor
    ledcWrite(pwmChannel, sinal_controle);

    // Calcula a velocidade
    calcula_velocidade_motor();

    // Valores enviados pela Serial
    //    Serial.print(sentido);
    //    Serial.print(',');
    if(sentido){
      Serial.println(sinal_controle);
    }

    else{
     Serial.println((-1)*sinal_controle);
      }
//    
//    Serial.print(',');
//    Serial.println(omega);
//    Serial.print(',');
//    Serial.println(angulo_motor);

    timer_prbs = millis();
  }
}

void calcula_velocidade_motor() {
  //
  noInterrupts();
  rpm = (pulsos_motor * (60.0 * 1000.0) / (taxaDeAmostragem * ENC_COUNT_REV_M));
  omega = (float) pulsos_motor * (1000 / taxaDeAmostragem) * 2 * PI / ENC_COUNT_REV_M ; // em rad/s

  pulsos_motor = 0;
  interrupts();
}

void verifica_canal_A() {
  if (digitalRead(ENC_A) == HIGH) {
    if (digitalRead(ENC_B) == LOW) {
      PontoAtual_enc_motor = PontoAtual_enc_motor + 1;
      pulsos_motor++;
    }
    else {
      PontoAtual_enc_motor = PontoAtual_enc_motor - 1;
      pulsos_motor--;
    }
  }
  else {
    if (digitalRead(ENC_B) == HIGH) {
      PontoAtual_enc_motor = PontoAtual_enc_motor + 1;
      pulsos_motor++;
    }
    else {
      PontoAtual_enc_motor = PontoAtual_enc_motor - 1;
      pulsos_motor--;
    }
  }
  angulo_motor = 2 * PI * PontoAtual_enc_motor / ENC_COUNT_REV_M;  // em radianos
}
void verifica_canal_B() {
  if (digitalRead(ENC_B) == HIGH) {
    if (digitalRead(ENC_A) == HIGH) {
      PontoAtual_enc_motor = PontoAtual_enc_motor + 1;
      pulsos_motor++;
    }
    else {
      PontoAtual_enc_motor = PontoAtual_enc_motor - 1;
      pulsos_motor--;
    }
  }
  else {
    if (digitalRead(ENC_A) == LOW) {
      PontoAtual_enc_motor = PontoAtual_enc_motor + 1;
      pulsos_motor++;
    }
    else {
      PontoAtual_enc_motor = PontoAtual_enc_motor - 1;
      pulsos_motor--;
    }
  }
  angulo_motor = 2 * PI * PontoAtual_enc_motor / ENC_COUNT_REV_M;  // em radianos
}