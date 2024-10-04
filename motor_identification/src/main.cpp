#include <Arduino.h>
#include <stdint.h>
#include <motor.h>
#include <encoder.h>
#include <stdlib.h>
#include <prbs.h>
#include <pins.h>

// Acionamentos
#define IDENT_BODE

// Constantes
#define SAMPLE_TIME 5 // ms - {velocidade = 10ms, angulo = 5ms}
#define AMPLITUDE 10 // Aplicada no sinal senoidal gerado para o motor
#define FREQUENCY_STEP 0 // Frequencia inicial a ser usada na geração do sinal senoidal
#define FREQUENCY_STEP_DELAY 4 // ms

void identificacao();
// uint16_t prbs(uint16_t* input, uint16_t bits, uint16_t stretch);

Motor motor(18,19,4);
Encoder encoder(ENC_A, ENC_B);

// Valor de referência 
int r = 0;
unsigned  int k = 0;
unsigned long frequency_counter = 0;

// Máximo valor de rotação em rad/s
int max_value = 255;

float u = 0.0;

// Variáveis do PRBS
static uint16_t stretch_counter = 0;
uint16_t bitsize = 14;
uint16_t i = random(0x01 << bitsize);
uint16_t stretch = 5;

void print_sensors(void* parameters) {
  Serial.print(u);
  Serial.print(",");
  Serial.print(encoder.get_omega());
  Serial.print(",");
  Serial.println(encoder.get_angular_position());
}

// Task para atualização de sensores
void update_sensors(void* parameters) {
  for(;;){
    // Qualquer sensor que precise executar atualização periodica pode ser colocado aqui
    encoder.update();

    vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS);
  }
}

// Task para identificação de posição
void identificacao(void* parameters) {
  for(;;){

    #ifndef IDENT_BODE
    r = map(analogRead(potPin), 0, 4095, -max_value, max_value);
  
    // Sinais PRBS
    u = (prbs(stretch_counter, &i, bitsize, stretch)-0.5) * 500;
    // prbs_rotation = prbs(&i, bitsize, stretch2);

    // Atuliza o comando para o motor
    motor.setSpeed(u);

    // Calcula a velocidade
    // omega = encoder.get_omega(SAMPLE_TIME);
    
    Serial.print(u);
    Serial.print(',');
    Serial.println(encoder.get_angular_position());
    #endif

    // Identificar diagrama de bode para velocidade
    #ifdef IDENT_BODE
    float frequency = frequency_counter * FREQUENCY_STEP;
    u = AMPLITUDE * sin(2*PI*frequency* k * SAMPLE_TIME / 1000);
    k++;
    motor.setSpeed(u);
    #endif

    vTaskDelay(SAMPLE_TIME / portTICK_PERIOD_MS);
  }
}

void increment_frequency(void* parameters){
  for(;;) {
    frequency_counter++;
    // Se estiver dando problema pq a dinamica ta muito lenta daria para
    // implementar algo mais rebuscado desativando essa task e utilizando
    // contador que espera alguns ciclos para começar a printagem pra ident
    // e depois de n amostras obtidas reabilitaria essa task
    vTaskDelay(FREQUENCY_STEP_DELAY / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(500000);
  pinMode(ENC_A , INPUT);
  pinMode(ENC_B , INPUT);
  pinMode(potPin, INPUT);

  // TODO: testar se o setup com interrupts funciona certo, senão retomar a declaração pra ca
  // attachInterrupt(digitalPinToInterrupt(ENC_A), std::bind(&Encoder::check_channel_A, &encoder), CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENC_B), std::bind(&Encoder::check_channel_B, &encoder), CHANGE);

  encoder.setup();
  motor.initMotor();

  xTaskCreate(
    print_sensors,
    "Print",
    1500, // Verificar quantidade de memoria necessaria - caso dê erro aumentar stack
    NULL,
    15, // Quão maior, maior será a prioridade
    NULL
  );

  xTaskCreatePinnedToCore(
    update_sensors,
    "Update",
    1500, // Verificar quantidade de memoria necessaria - caso dê erro aumentar stack
    NULL,
    15, // Quão maior, maior será a prioridade
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    identificacao,
    "Print",
    3000, // Verificar quantidade de memoria necessaria - caso dê erro aumentar stack
    NULL,
    15, // Quão maior, maior será a prioridade
    NULL,
    1
  );

  #ifdef IDENT_BODE
  xTaskCreate(
    increment_frequency,
    "Print",
    1000, // Verificar quantidade de memoria necessaria - caso dê erro aumentar stack
    NULL,
    15, // Quão maior, maior será a prioridade
    NULL
  );
  #endif

  Serial.println("End of Setup");
}

void loop(){
  // identificacao();
}