#ifndef ENCODER_H
#define ENCODER_H

class Encoder
{
private:
    float omega = 0.0;
    float rpm = 0.0;

public:
    Encoder();
    ~Encoder(); 

    /**
     * Este método retorna a rotação do motor em rad/s.
     * @param sampleTime é a taxa de amostragem para o cálculo da rotação do motor
     */
    float get_omega(unsigned long sampleTime);

    // Captura o sentido de rotação do motor
    bool direcao = true;

    // Quantidade de pulsos por revolução do motor
    const int ENC_COUNT_REV = 1368.4;
    
    // Variável para a contagem dos pulsos
    volatile long pulses_motor = 0;

    // Variável para capturar a posição angular
    signed long ActualPoint_enc_motor = 0;
};


#endif
