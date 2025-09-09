#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include "ssd1306.h" // Inclua a biblioteca do OLED da BitDogLab

// Definições de pinos
#define JOY_X 27      // GPIO27 -> ADC1
#define JOY_Y 26      // GPIO26 -> ADC0
#define ADC_FILTER 28 // GPIO28 -> ADC2 (saída do filtro RC)
#define PWM_PIN 16    // Saída PWM

#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3

// Variáveis globais
uint slice_num;
static uint16_t pwm_wrap = 0; // wrap atual do PWM

// Função para inicializar PWM
void pwm_init_custom(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_enabled(slice_num, true);
}

// Função para configurar frequência do PWM
void pwm_set_freq(uint freq) {
    uint32_t clock = 125000000; // Clock padrão do PWM (125 MHz)

    // Calcula divisor
    uint32_t divider16 = clock / freq / 4096 + (clock % (freq * 4096) != 0);
    if (divider16 / 16 == 0) divider16 = 16;

    // Calcula wrap
    pwm_wrap = clock * 16 / divider16 / freq - 1;

    pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice_num, pwm_wrap);
}

// Função para configurar duty cycle (0–100%)
void pwm_set_duty(uint duty_percent) {
    uint16_t level = (pwm_wrap * duty_percent) / 100;
    pwm_set_gpio_level(PWM_PIN, level);
}

int main() {
    stdio_init_all();

    // Inicializa ADC
    adc_init();
    adc_gpio_init(JOY_X);
    adc_gpio_init(JOY_Y);
    adc_gpio_init(ADC_FILTER);

    // Inicializa PWM
    pwm_init_custom(PWM_PIN);

    // Inicializa OLED
    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    ssd1306_t disp;
    disp.external_vcc=false;
    ssd1306_init(&disp, 128, 64, 0x3C, i2c1);

    while (true) {
        // Leitura dos valores do joystick (12 bits → 0–4095)
        adc_select_input(0); // eixo Y
        uint16_t val_y = adc_read();
        adc_select_input(1); // eixo X
        uint16_t val_x = adc_read();

        // Converte para duty e frequência
        uint duty = (val_x * 100) / 4095;                 // 0–100 %
        uint fc = 1000;
        uint freq = 100 + (val_y * (200000-100)) / 4095; // 100 Hz – 20 kHz

        // Aplica no PWM
        pwm_set_freq(freq);
        pwm_set_duty(duty);

        // Lê a saída do filtro RC
        adc_select_input(2);
        uint16_t val_adc = adc_read();
        float v_out = (val_adc * 3.3f) / 4095.0f;

        // Valor esperado
        float v_expected = (duty / 100.0f) * 3.3f;

        // Mostra resultados no terminal
        printf("Duty: %u %% | Freq: %u Hz | Vmed: %.2f V | Vexp: %.2f V\n",
               duty, freq, v_out, v_expected);

        // --- Mostra resultados no OLED ---
        ssd1306_clear(&disp);
        char buf[32];
        snprintf(buf, sizeof(buf), "Duty: %u %%", duty);
        ssd1306_draw_string(&disp, 0, 0,1, buf);
        snprintf(buf, sizeof(buf), "Freq: %u Hz", freq);
        ssd1306_draw_string(&disp, 0, 10,1, buf);
        snprintf(buf, sizeof(buf), "Vmed: %.2f V", v_out);
        ssd1306_draw_string(&disp, 0, 20, 1,buf);
        snprintf(buf, sizeof(buf), "Vexp: %.2f V", v_expected);
        ssd1306_draw_string(&disp,0, 30,1, buf);
        ssd1306_show(&disp);

        sleep_ms(200);
    }
}
