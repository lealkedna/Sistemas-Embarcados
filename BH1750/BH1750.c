#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

#define BH1750_ADDR 0x23
#define BH1750_CMD_CONT_HRES_MODE 0x10

void bh1750_init() {
    uint8_t cmd = BH1750_CMD_CONT_HRES_MODE;
    i2c_write_blocking(I2C_PORT, BH1750_ADDR, &cmd, 1, false);
}

float bh1750_read_lux() {
    uint8_t data[2];
    i2c_read_blocking(I2C_PORT, BH1750_ADDR, data, 2, false);
    uint16_t raw = (data[0] << 8) | data[1];
    return raw / 1.2;  // Conversão para lux
}

int main() {
    stdio_init_all();
    sleep_ms(2000);  

    // Inicializa o I2C
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicia o BH1750
    bh1750_init();

    printf("Iniciando leitura de luminosidade...\n");

    while (true) {
        float lux = bh1750_read_lux();
        printf("Luminosidade: %.2f lux\n", lux);
        sleep_ms(1000);
    }
}