#pragma once

#include <Arduino.h>
#include <FunctionalInterrupt.h>

class Encoder
{
private:
    float omega = 0.0;
    float rpm = 0.0;
    unsigned long timer;
    uint8_t channel_A, channel_B;

public:
    
    Encoder(uint8_t channel_A, uint8_t channel_B);
    ~Encoder(); 

    /*
    * Este método inicializa os pinos de interrupção para o encoder
    */
    void setup();

    /*
    * Este método deve ser chamado periodicamente para atualizar os dados do encoder
    */
    void update();

    /**
     * Este método retorna a rotação do motor em rad/s.
     */
    float get_omega();
    /*
    * Este método retorna a posição angular do motor em radianos
    */
    float get_angular_position();

    /*
    * Este método é chamado pela interrupção do canal A do encoder
    *  - IIRAM_ATTR: Método sempre será executado da memoria RAM
    */
    void IRAM_ATTR check_channel_A();

    /*
    * Este método é chamado pela interrupção do canal B do encoder
    *  - IIRAM_ATTR: Método sempre será executado da memoria RAM
    */
    void IRAM_ATTR check_channel_B();

    // Captura o sentido de rotação do motor
    bool direcao = true;

    // Quantidade de pulsos por revolução do motor
    const int ENC_COUNT_REV = 1368.4;
    
    // Variável para a contagem dos pulsos
    volatile long pulses_motor = 0;

    // Variável para capturar a posição angular
    signed long ActualPoint_enc_motor = 0;
};