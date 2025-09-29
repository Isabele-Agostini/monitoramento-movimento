// servo.c

#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

// --- Constantes para o servo motor SG90 ---

// Frequência do PWM em Hz
#define PWM_FREQUENCY 50 

// Largura de pulso mínima em microssegundos (correspondente a 0 graus)
#define MIN_PULSE_US 500

// Largura de pulso máxima em microssegundos (correspondente a 180 graus)
#define MAX_PULSE_US 2500

// Período total do ciclo PWM em microssegundos (1 / 50Hz = 0.02s = 20000 us)
#define PWM_WRAP_VALUE 19999 

void servo_init(uint gpio_pin) {
    // Configura o pino GPIO para usar a função PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);

    // Encontra qual "slice" do PWM controla este pino
    // CORREÇÃO 1: A função correta é pwm_gpio_to_slice_num()
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);

    // Obtém a configuração padrão do PWM
    pwm_config config = pwm_get_default_config();
    
    // Calcula o divisor de clock para ter uma resolução de 1us
    // O clock do sistema é geralmente 125 MHz (125,000,000 Hz)
    // 125,000,000 / 1,000,000 = 125. Isso faz o contador do PWM incrementar a cada 1us.
    float clk_div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    pwm_config_set_clkdiv(&config, clk_div);

    // Define o valor de "wrap". O contador do PWM irá de 0 até este valor.
    // Com um clock de 1MHz (1us por contagem), 20000 contagens levam 20000us ou 20ms,
    // o que corresponde a uma frequência de 50Hz (1 / 0.020s).
    pwm_config_set_wrap(&config, PWM_WRAP_VALUE);

    // Aplica a configuração ao slice do PWM e o inicia
    pwm_init(slice_num, &config, true);
}

void servo_set_pulse_width(uint gpio_pin, uint32_t pulse_width_us) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel_num = pwm_gpio_to_channel(gpio_pin);

    // Define o nível do canal, que corresponde diretamente à largura do pulso em us
    // devido à nossa configuração de clock
    // CORREÇÃO 2: A função correta é pwm_set_chan_level()
    pwm_set_chan_level(slice_num, channel_num, pulse_width_us);
}

void servo_set_angle(uint gpio_pin, float angle) {
    // Garante que o ângulo está dentro do intervalo permitido (0-180)
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // Mapeia o ângulo (0-180) para a largura de pulso (MIN_PULSE_US - MAX_PULSE_US)
    uint32_t pulse_width = MIN_PULSE_US + (uint32_t)((angle / 180.0f) * (MAX_PULSE_US - MIN_PULSE_US));
    
    servo_set_pulse_width(gpio_pin, pulse_width);
}