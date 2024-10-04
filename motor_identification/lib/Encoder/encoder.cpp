#include "encoder.h" 

Encoder::Encoder(uint8_t channel_A, uint8_t channel_B){
  this->channel_A = channel_A;
  this->channel_B = channel_B;
}

Encoder::~Encoder(){}

void Encoder::setup(){
  attachInterrupt(digitalPinToInterrupt(channel_A), std::bind(&Encoder::check_channel_A, this), CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_B), std::bind(&Encoder::check_channel_B, this), CHANGE);
}

void Encoder::update(){
  unsigned long ts = millis() - timer;

  this->rpm = (pulses_motor * (60.0 * 1000.0) / (ts * ENC_COUNT_REV));
  this->omega = (float) pulses_motor * (1000 / ts) * 2 * 3.141592 / ENC_COUNT_REV ; // em rad/s

  pulses_motor = 0;
  timer = millis();
}

float Encoder::get_omega(){
  return this->omega;
};

float Encoder::get_angular_position(){
  return 2 * PI * ActualPoint_enc_motor / ENC_COUNT_REV;  // em radianos
}

void Encoder::check_channel_A(){
  if (digitalRead(channel_A) == HIGH) {
    if (digitalRead(channel_B) == LOW) {
      ActualPoint_enc_motor++;
      pulses_motor++;
    }
    else {
      ActualPoint_enc_motor--;
      pulses_motor--;
    }
  }
  else {
    if (digitalRead(channel_B) == HIGH) {
      ActualPoint_enc_motor++;
      pulses_motor++;
    }
    else {
      ActualPoint_enc_motor--;
      pulses_motor--;
    }
  }

    // angle_motor = 2 * PI * encoder.ActualPoint_enc_motor / encoder.ENC_COUNT_REV;  // em radianos
}

void Encoder::check_channel_B(){
  if (digitalRead(channel_B) == HIGH) {
    if (digitalRead(channel_A) == HIGH) {
      ActualPoint_enc_motor++;
      pulses_motor++;
    }
    else {
      ActualPoint_enc_motor--;
      pulses_motor--;
    }
  }
  else {
    if (digitalRead(channel_A) == LOW) {
      ActualPoint_enc_motor++;
      pulses_motor++;
    }
    else {
      ActualPoint_enc_motor--;
      pulses_motor--;
    }
  }

  // angle_motor = 2 * PI * encoder.ActualPoint_enc_motor / encoder.ENC_COUNT_REV;  // em radianos
}