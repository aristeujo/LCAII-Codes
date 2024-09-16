#include <Arduino.h>
#include <stdint.h>
#include <motor.h>
#include <encoder.h>

void check_channel_A();
void check_channel_B();
void identificacao();
uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch);

Motor motor(18,19,4);
Encoder encoder;

// Porta utilizado para o sinal do potenciômetro
#define potPin  34

// Portas dos canais do encoder
#define ENC_A 12 // fio verde
#define ENC_B 13 // fio amarelo

// Variável de controle de tempo
// int sampleTime = 10; // para velocidade
int sampleTime = 5; // para angulo
long timer = 0;

// Valor da posição angular do motor em radiano
float angle_motor = 0;

float omega = 0;

// Valor de referência 
int r = 0;

// Máximo valor de rotação em rad/s
int max_value = 255;

float u = 0.0;

// Variáveis do PRBS
static uint16_t stretch_counter = 0;
uint16_t bitsize = 14;
uint16_t i = random(0x01 << bitsize);
uint16_t stretch = 5;

void setup() {
  Serial.begin(500000);
  pinMode(ENC_A , INPUT);
  pinMode(ENC_B , INPUT);
  pinMode(potPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), check_channel_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), check_channel_B, CHANGE);

  motor.initMotor();

  Serial.println("End of Setup");
}

void loop() {
  identificacao();
}

void identificacao() {
  if ((millis() - timer) >= sampleTime) {

  r = map(analogRead(potPin), 0, 4095, -max_value, max_value);
  
  // Sinais PRBS
  u = (prbs(&i, bitsize, stretch)-0.5) * 500;
  // prbs_rotation = prbs(&i, bitsize, stretch2);

  // Atuliza o comando para o motor
  motor.setSpeed(u);

  // Calcula a velocidade
  // omega = encoder.get_omega(sampleTime);
  
  Serial.print(u);
  Serial.print(',');
  Serial.println(angle_motor);

  timer = millis();
  }
}

uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch)
{
  if (stretch_counter >= stretch) {
    stretch_counter = 0; // Reset counter

    uint16_t mask = (1 << bits) - 1;
    uint16_t comp = (*input ^ (*input >> 1)) & 1;
    *input = ((*input >> 1) | (comp << (bits - 1))) & mask; // Bit-shifting and adding the XOR result
  } else {
    stretch_counter += 1;
  }
  return *input & 0x01;
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