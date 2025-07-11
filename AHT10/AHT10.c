#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C0 para AHT10 (sensor)
#define I2C_SENSOR i2c0
#define SDA_SENSOR 0
#define SCL_SENSOR 1
#define AHT10_ADDR 0x38

// Inicializa o sensor AHT10
void aht10_init() {
    uint8_t init_cmd[3] = {0xE1, 0x08, 0x00};
    i2c_write_blocking(I2C_SENSOR, AHT10_ADDR, init_cmd, 3, false);
    sleep_ms(50);
}

// Lê temperatura e umidade; retorna true se leitura válida
bool aht10_read(float *temp_c, float *umid_pct) {
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t data[6];

    i2c_write_blocking(I2C_SENSOR, AHT10_ADDR, cmd, 3, false);
    sleep_ms(80);
    int result = i2c_read_blocking(I2C_SENSOR, AHT10_ADDR, data, 6, false);

    if (result != 6 || (data[0] & 0x80)) {
        return false; // Leitura incompleta ou sensor ocupado
    }

    // Umidade: 20 bits
    uint32_t raw_humid = ((uint32_t)data[1] << 16) |
                         ((uint32_t)data[2] << 8) |
                         data[3];
    raw_humid = raw_humid >> 4;
    *umid_pct = (raw_humid * 100.0f) / 1048576.0f;

    // Temperatura: 20 bits
    uint32_t raw_temp = (((uint32_t)(data[3] & 0x0F)) << 16) |
                        ((uint32_t)data[4] << 8) |
                        data[5];
    *temp_c = ((raw_temp * 200.0f) / 1048576.0f) - 50.0f;

    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    // Inicializa I2C do sensor
    i2c_init(I2C_SENSOR, 100 * 1000);
    gpio_set_function(SDA_SENSOR, GPIO_FUNC_I2C);
    gpio_set_function(SCL_SENSOR, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_SENSOR);
    gpio_pull_up(SCL_SENSOR);

    aht10_init();

    while (true) {
        float temp, umi;
        bool ok = aht10_read(&temp, &umi);

        if (ok) {
            printf("Temperatura: %.2f °C | Umidade: %.2f %%\n", temp, umi);

            if (temp < 20.0 && umi > 70.0) {
                printf("⚠️  ALERTA: Frio e umidade alta!\n");
            } else if (temp < 20.0) {
                printf("⚠️  ALERTA: Temperatura abaixo de 20 °C!\n");
            } else if (umi > 70.0) {
                printf("⚠️  ALERTA: Umidade acima de 70%%!\n");
            }
        } else {
            printf("❌ Erro ao ler o sensor AHT10. Verifique conexões.\n");
        }

        printf("------------------------------\n");
        sleep_ms(2000);
    }

    return 0;
}
