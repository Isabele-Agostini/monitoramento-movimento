// servo.h

#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"

/**
 * @brief Inicializa o sistema PWM para um pino GPIO específico para controlar um servo.
 * * Configura o pino GPIO para a função PWM e ajusta o clock do PWM para
 * operar a 50 Hz, a frequência padrão para servos SG90.
 *
 * @param gpio_pin O número do pino GPIO ao qual o servo está conectado.
 */
void servo_init(uint gpio_pin);

/**
 * @brief Define o ângulo do servo motor.
 *
 * Converte o ângulo desejado (0 a 180 graus) em uma largura de pulso em microssegundos
 * e ajusta o nível do canal PWM correspondente.
 *
 * @param gpio_pin O número do pino GPIO do servo a ser controlado.
 * @param angle O ângulo desejado, em graus (de 0.0 a 180.0).
 */
void servo_set_angle(uint gpio_pin, float angle);

/**
 * @brief Define a largura do pulso do PWM diretamente em microssegundos.
 *
 * Função de nível mais baixo para controle direto, útil para calibrar
 * ou usar servos com especificações diferentes. Para um SG90, o intervalo
 * típico é de 500 a 2500 microssegundos.
 *
 * @param gpio_pin O número do pino GPIO do servo a ser controlado.
 * @param pulse_width_us A largura do pulso em microssegundos.
 */
void servo_set_pulse_width(uint gpio_pin, uint32_t pulse_width_us);

#endif // SERVO_H