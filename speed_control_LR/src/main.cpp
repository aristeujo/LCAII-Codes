#include <Arduino.h>
#include <stdint.h>
#include "motor.h"
#include "encoder.h"

void check_channel_A();
void check_channel_B();
void diff_eq_control();

Motor motor(18,19,4);
Encoder encoder;

// Porta utilizado para o sinal do potenciômetro
#define potPin  34

// Portas dos canais do encoder
#define ENC_A 12 // fio verde
#define ENC_B 13 // fio amarelo

// Variável de controle de tempo
int sampleTime = 35;
long timer = 0;

// Valor da posição angular do motor em radiano
float angle_motor = 0;

// Valor de referência 
int r = 0;

// Máximo valor de rotação em rad/s
int max_value = 30;

const int ordem = 3;   // Definir a ordem do seu controlador

float e[ordem] = {}; // Coeficientes na seguinte ordem e[n] = {a*e[k-1], b*e[k-2], ..., n*e[k-n]} com n = ordem
float u[ordem] = {};  // Coeficientes na seguinte ordem u[m] = {x*u[k-1], y*u[k-2], ..., z*e[k-m]} com m = ordem

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A , INPUT);
  pinMode(ENC_B , INPUT);
  pinMode(potPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), check_channel_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), check_channel_B, CHANGE);

  motor.initMotor();

  Serial.println("End of Setup");
}

void loop() {
  diff_eq_control();
}

// Função onde é implementado o controle por equação de diferenças
void diff_eq_control() {
  r = map(analogRead(potPin), 0, 4095, 0, max_value);

  if (millis() - timer >= sampleTime) {

    float omega = encoder.get_omega(sampleTime);

    for (int i = 2; i > 0; i--) {
      e[i] = e[i - 1];
      u[i] = u[i - 1];
    }

    // Cálculo do erro
    e[0] = r - omega;

    // Cálculo do sinal de controle
    u[0] = 1.1329 * u[1] - 0.1329 * u[2] + 6.7469 * e[1] - 6.0454 * e[2];

    u[0] = constrain(u[0], 0, 255); // saturação do sinal de controle

    // Aciona o motor
    motor.setSpeed(u[0]);

    Serial.print(omega);
    Serial.print(",");
    Serial.println(r);

    timer = millis();
  }
}

/**
* Função de interrupção para contar os pulsos do motor no canal A
*/
void check_channel_A() {
  if (digitalRead(ENC_A) == HIGH) {
    if (digitalRead(ENC_B) == LOW) {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor + 1;
      encoder.pulses_motor++;
    }
    else {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor - 1;
      encoder.pulses_motor--;
    }
  }
  else {
    if (digitalRead(ENC_B) == HIGH) {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor + 1;
      encoder.pulses_motor++;
    }
    else {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor - 1;
      encoder.pulses_motor--;
    }
  }

  angle_motor = 2 * PI * encoder.ActualPoint_enc_motor / encoder.ENC_COUNT_REV;  // em radianos
}

/**
* Função de interrupção para contar os pulsos do motor no canal B
*/
void check_channel_B() {
  if (digitalRead(ENC_B) == HIGH) {
    if (digitalRead(ENC_A) == HIGH) {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor + 1;
      encoder.pulses_motor++;
    }
    else {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor - 1;
      encoder.pulses_motor--;
    }
  }
  else {
    if (digitalRead(ENC_A) == LOW) {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor + 1;
      encoder.pulses_motor++;
    }
    else {
      encoder.ActualPoint_enc_motor = encoder.ActualPoint_enc_motor - 1;
      encoder.pulses_motor--;
    }
  }

  angle_motor = 2 * PI * encoder.ActualPoint_enc_motor / encoder.ENC_COUNT_REV;  // em radianos
}