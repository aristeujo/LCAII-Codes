#include <Arduino.h>
#include <stdint.h>
#include <motor.h>
#include <encoder.h>

void check_channel_A();
void check_channel_B();
uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch);

Motor motor(18,19,4);
Encoder encoder;

// Variáveis para aleatoriedade
int range = 0;
int count = 0;
unsigned seed;

// Variáveis do PRBS
uint16_t stretch_counter = 0;
uint16_t bitsize = 6;
uint16_t i = random(0x01<<bitsize);
uint16_t s = 3;

// Portas dos canais do encoder
#define ENC_A 12 // fio verde
#define ENC_B 13 // fio amarelo

// Variável de controle de tempo
int sampleTime = 35;
long timer = 0;

// Valor da posição angular do motor em radiano
float angle_motor = 0;

// Sinal de controle
int u = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENC_A , INPUT);
  pinMode(ENC_B , INPUT);
  pinMode(36, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), check_channel_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), check_channel_B, CHANGE);

  motor.initMotor();
  seed = analogRead(36);
  range = random(0, 96);

  Serial.println("End of Setup");
}

void loop() {
  if ((millis() - timer) >= sampleTime) {
    seed = analogRead(36);
    srandom(seed);
    count++;

    // Flag para segurar o ultimo valor por 3 amostras
    if (count == 3) {
      range = random(0, 96);
      count = 0;
    }

    u = range + 150;  // em degraus
    // u = (prbs(&i, bitsize, s)) * 80 + 150;

    motor.setSpeed(u);
    float omega = encoder.get_omega(sampleTime);

    Serial.println(u);
    Serial.print(",");
    Serial.println(omega);
  }
}

uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch) {
  if (stretch_counter >= stretch) {
    stretch_counter = 0;  // Reset counter

    uint16_t mask = (0x01 << bits) - 1;
    uint16_t comp = (*input ^ (*input >> 1)) << (bits - 1);  // XOR'ing the last 2 numbers (LSB)
    comp = comp & (0x01 << (bits - 1));
    *input = ((*input >> 1) | comp) & mask;  // Bit-shifting and adding the XOR result
  } else {
    stretch_counter += 1;
  }
  return *input & 0x01;
};


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