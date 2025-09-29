#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Bloco de inclusão para bibliotecas em C
extern "C" {
#include "servo.h"
#include "ssd1306.h"
}

// Inclui biblioteca do sensor MPU6050
#include "MPU6050.h"

// --- PARÂMETROS DE AJUSTE DO SISTEMA ---

// --- Configurações do Display OLED (i2c1) ---
#define I2C1_PORT i2c1
#define I2C1_SDA_PIN 14
#define I2C1_SCL_PIN 15
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDR 0x3C

// --- Configurações do Acelerômetro MPU6050 (i2c0) ---
#define I2C0_PORT i2c0
#define I2C0_SDA_PIN 0
#define I2C0_SCL_PIN 1
#define MPU6050_ADDR 0x68

const float SENSIBILIDADE_ACELEROMETRO = 16384.0f;

// --- Configuração do Atuador Servo ---
#define PINO_SERVO 2

// Velocidade de operação das interfaces I2C
#define VELOCIDADE_I2C 400000

// Parâmetros de controle do servo rotativo contínuo
const float LIMIAR_ALERTA_ANGULAR = 45.0f;
#define PULSO_PARADA_SERVO_US 1500
#define PULSO_MINIMO_SERVO_US 500
#define PULSO_MAXIMO_SERVO_US 2500
#define ZONA_MORTA_GRAUS 5.0f

float converter_valor(float valor, float deMin, float deMax, float paraMin, float paraMax) {
    return (valor - deMin) * (paraMax - paraMin) / (deMax - deMin) + paraMin;
}

// --- Principal ---
int main() {
    stdio_init_all();
    sleep_ms(2000);

    printf("Sistema de Controle de Inclinação - Versão 2.0\n");
    printf("Inicializando componentes...\n");

    // --- Configuração da Interface I2C0 para Acelerômetro ---
    i2c_init(I2C0_PORT, VELOCIDADE_I2C);
    gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C0_SDA_PIN);
    gpio_pull_up(I2C0_SCL_PIN);
    printf("Interface I2C0 configurada para MPU6050 - Pinos: Dados=%d, Clock=%d\n", 
           I2C0_SDA_PIN, I2C0_SCL_PIN);

    // --- Configuração da Interface I2C1 para Display ---
    i2c_init(I2C1_PORT, VELOCIDADE_I2C);
    gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C1_SDA_PIN);
    gpio_pull_up(I2C1_SCL_PIN);
    printf("Interface I2C1 configurada para OLED - Pinos: Dados=%d, Clock=%d\n", 
           I2C1_SDA_PIN, I2C1_SCL_PIN);

    // Inicializa sensor MPU6050 na interface I2C0
    MPU6050 mpu(I2C0_PORT, MPU6050_ADDR);
    mpu.begin();
    printf("Sensor MPU6050 inicializado com sucesso. Identificador: 0x%X\n", mpu.getId());
    
    // Configura servo motor no pino definido
    servo_init(PINO_SERVO);
    printf("Atuador servo motor configurado no pino GPIO%d.\n", PINO_SERVO);

    // Prepara display OLED na interface I2C1
    ssd1306_t disp;
    disp.external_vcc = false; // <-- Utiliza alimentação interna
    ssd1306_init(&disp, OLED_WIDTH, OLED_HEIGHT, OLED_ADDR, I2C1_PORT);
    printf("Display OLED SSD1306 pronto para operação.\n");
    
    MPU6050::VECT_3D aceleracao;
    float inclinacao;
    uint32_t largura_pulso_us;

    printf("\n=== Iniciando Loop Principal de Controle ===\n");

    while (1) {
        // Aquisição de dados do sensor
        mpu.getAccel(&aceleracao, SENSIBILIDADE_ACELEROMETRO);
        inclinacao = atan2(-aceleracao.x, sqrt(aceleracao.y * aceleracao.y + 
                         aceleracao.z * aceleracao.z)) * 180.0 / M_PI;
        
        printf("Ângulo de Inclinação: %6.2f° | ", inclinacao);

        // Lógica de controle do servo
        if (fabs(inclinacao) < ZONA_MORTA_GRAUS) {
            largura_pulso_us = PULSO_PARADA_SERVO_US;
            printf("Servo: PARADO");
        } else {
            largura_pulso_us = (uint32_t)converter_valor(inclinacao, -90.0f, 90.0f, 
                                                  PULSO_MAXIMO_SERVO_US, PULSO_MINIMO_SERVO_US);

            // Aplicação de limites de segurança
            if (largura_pulso_us < PULSO_MINIMO_SERVO_US) largura_pulso_us = PULSO_MINIMO_SERVO_US;
            if (largura_pulso_us > PULSO_MAXIMO_SERVO_US) largura_pulso_us = PULSO_MAXIMO_SERVO_US;
            
            printf("Servo: ATUANDO");
        }

        // Envio de comando para o servo
        servo_set_pulse_width(PINO_SERVO, largura_pulso_us);
        printf(" | Largura do Pulso: %4u μs\n", largura_pulso_us);

        // Atualização do display OLED
        ssd1306_clear(&disp);
        char texto_display[20];
        
        if (fabs(inclinacao) > LIMIAR_ALERTA_ANGULAR) {
            // Modo de alerta - inclinação excessiva
            ssd1306_draw_string(&disp, 25, 16, 2, "ALERTA!");
            snprintf(texto_display, sizeof(texto_display), "%.1f graus", inclinacao);
            ssd1306_draw_string(&disp, 15, 40, 1, texto_display);
        } else {
            // Modo normal - exibe velocidade do servo
            ssd1306_draw_string(&disp, 0, 16, 1, "Controle Servo:");
            snprintf(texto_display, sizeof(texto_display), "%u μs", largura_pulso_us);
            ssd1306_draw_string(&disp, 20, 35, 2, texto_display);
        }
        ssd1306_show(&disp);

        // Intervalo de amostragem
        sleep_ms(100);
    }

    return 0;
}