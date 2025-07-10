#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "vl53l0x_api.h"
#include "ssd1306.h"
// Pinos LEDs do semáforo
#define BLUE_LED_PIN 12
#define RED_LED_PIN 13
#define GREEN_LED_PIN 11

// Buzzer
#define BUZZER_PIN 21
#define BUZZER_FREQUENCY 100

// I2C0 - Sensor de distância
#define SDA_VL53L0X 0
#define SCL_VL53L0X 1

// I2C1 - Display OLED
#define SDA_OLED 14
#define SCL_OLED 15

// Instância do sensor
VL53L0X_Dev_t dev;
VL53L0X_DEV dev_ptr = &dev;
void pwm_init_buzzer(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_get_hz(clk_sys) / (BUZZER_FREQUENCY * 4096));
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(pin, 0);
}

void beep(uint pin, uint duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(pin);
    pwm_set_gpio_level(pin, 2048);
    sleep_ms(duration_ms);
    pwm_set_gpio_level(pin, 0);
}


void init_vl53l0x() {
    i2c_init(i2c0, 400000);
    gpio_set_function(SDA_VL53L0X, GPIO_FUNC_I2C);
    gpio_set_function(SCL_VL53L0X, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_VL53L0X);
    gpio_pull_up(SCL_VL53L0X);

    dev_ptr->I2cDevAddr = 0x29;
    dev_ptr->comms_type = 1;
    dev_ptr->comms_speed_khz = 400;

    VL53L0X_DataInit(dev_ptr);
    VL53L0X_StartMeasurement(dev_ptr);
}


void init_oled_and_leds(uint8_t *buf, struct render_area *area) {
    i2c_init(i2c1, 400000);
    gpio_set_function(SDA_OLED, GPIO_FUNC_I2C);
    gpio_set_function(SCL_OLED, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_OLED);
    gpio_pull_up(SCL_OLED);

    SSD1306_init();

    area->start_col = 0;
    area->end_col = SSD1306_WIDTH - 1;
    area->start_page = 0;
    area->end_page = SSD1306_NUM_PAGES - 1;
    calc_render_area_buflen(area);

    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, area);

    gpio_init(BLUE_LED_PIN);
    gpio_init(RED_LED_PIN);
    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(BLUE_LED_PIN, GPIO_OUT);
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
}


void show_text(uint8_t *buf, struct render_area *area, char *text[]) {
    memset(buf, 0, SSD1306_BUF_LEN);
    int y = 0;
    for (uint i = 0; i < 4; i++) {
        WriteString(buf, 5, y, text[i]);
        y += 8;
    }
    render(buf, area);
}



int main() {
    stdio_init_all();
    sleep_ms(2000);

    uint8_t buf[SSD1306_BUF_LEN];
    struct render_area area;
    init_oled_and_leds(buf, &area);
    init_vl53l0x();
    pwm_init_buzzer(BUZZER_PIN);

    while (true) {
        VL53L0X_RangingMeasurementData_t m;
        VL53L0X_PerformSingleRangingMeasurement(dev_ptr, &m);

        if (m.RangeMilliMeter < 100 && m.RangeMilliMeter > 0) {
            // ALERTA ATIVO
            gpio_put(RED_LED_PIN, 1);
            gpio_put(GREEN_LED_PIN, 0);
            gpio_put(BLUE_LED_PIN, 0);
            beep(BUZZER_PIN, 300);

            char *msg[] = {
                "Objeto",
                "    Detectado ",
                "  A menos 10 CM"};
            show_text(buf, &area, msg);
        } else {
            // SEMÁFORO NORMAL
            gpio_put(BLUE_LED_PIN, 1);
            gpio_put(RED_LED_PIN, 0);
            gpio_put(GREEN_LED_PIN, 0);

            char *msg[] = {
                "Sem objeto proximo"};
            show_text(buf, &area, msg);
        }

        sleep_ms(1000);
    }
}

// int main()
// {
//     stdio_init_all();
//     setup_audio();

//     // I2C é "open drain", usamos pull-ups para manter o sinal alto quando não há dados sendo transmitidos
//     i2c_init(i2c1, SSD1306_I2C_CLK * 1000);
//     gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA_PIN);
//     gpio_pull_up(I2C_SCL_PIN);

//     SSD1306_init();

//     struct render_area frame_area = {
//         start_col : 0,
//         end_col : SSD1306_WIDTH - 1,
//         start_page : 0,
//         end_page : SSD1306_NUM_PAGES - 1
//     };
//     calc_render_area_buflen(&frame_area);

//     uint8_t buf[SSD1306_BUF_LEN];
//     memset(buf, 0, SSD1306_BUF_LEN);
//     render(buf, &frame_area);

//     // Inicializa os LEDs
//     gpio_init(BLUE_LED_PIN);
//     gpio_init(RED_LED_PIN);
//     gpio_init(GREEN_LED_PIN);
//     gpio_set_dir(BLUE_LED_PIN, GPIO_OUT);
//     gpio_set_dir(RED_LED_PIN, GPIO_OUT);
//     gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);

//     gpio_put(BLUE_LED_PIN, 1);  // Acende o LED azul inicialmente
//     gpio_put(RED_LED_PIN, 0);   // Desliga o LED vermelho
//     gpio_put(GREEN_LED_PIN, 0); // Desliga o LED verde

                     

//     uint32_t time_started = time_us_32(); // Captura o tempo de início
//     uint32_t last_button_press = 0;      // Variável para debounce
//     bool button_pressed = false;

//     while (true)
// {
//     uint32_t elapsed_time = time_us_32() - time_started; // Tempo decorrido

//     // Exibe a mensagem de "Simulação" nos primeiros 15 segundos
//     if (elapsed_time < 15000000) // 15 segundos em microsegundos
//     {
//         char *text[] = {
//             "   Simula ",
//             "           ",
//             " o semaforo ",
//             "            "};

//         int y = 0;
//         for (uint i = 0; i < count_of(text); i++)
//         {
//             WriteString(buf, 5, y, text[i]);
//             y += 8;
//         }
//         render(buf, &frame_area);
//     }
//     else if (elapsed_time < 30000000) // Após 15 segundos, LED vermelho acende por 15 segundos
//     {
//         // Após 15 segundos, muda o LED para vermelho e texto para "Espere..."
//         gpio_put(BLUE_LED_PIN, 0);  // Desliga o LED azul
//         gpio_put(RED_LED_PIN, 1);   // Acende o LED vermelho
//         gpio_put(GREEN_LED_PIN, 0); // Desliga o LED verde

//         char *text[] = {
//             "         ",
//             " Espere  ",
//             "         ",
//             "        "};

//         int y = 0;
//         for (uint i = 0; i < count_of(text); i++)
//         {
//             WriteString(buf, 5, y, text[i]);
//             y += 8;
//         }
//         render(buf, &frame_area);
//     }
//     else // Após 30 segundos, LED verde acende e texto muda para "Atravesse"
//     {
//         // Após 30 segundos, muda o LED para verde e texto para "Atravesse"
//         gpio_put(RED_LED_PIN, 0);   // Desliga o LED vermelho
//         gpio_put(GREEN_LED_PIN, 1); // Acende o LED verde

//         char *text[] = {
//             "         ",
//             "Atravesse ",
//             "         ",
//             "        "};

//         int y = 0;
//         for (uint i = 0; i < count_of(text); i++)
//         {
//             WriteString(buf, 5, y, text[i]);
//             y += 8;
//         }
//         render(buf, &frame_area);
//     }

//     sleep_ms(500); // Espera um pouco antes de atualizar
// }


//     return 0;
// }
