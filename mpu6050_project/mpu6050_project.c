#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU6050_ADDR 0x68

int main()
{
    stdio_init_all();

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    sleep_ms(500);
    // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c
    uint8_t config[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, config, 2, false);

    while (true)
    {
        // printf("Hello, world!\n");
        // sleep_ms(1000);
        uint8_t reg = 0x3B;
        uint8_t data[14];
        i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
        i2c_read_blocking(I2C_PORT, MPU6050_ADDR, data, 14, false);

        // Converte dados brutos
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        int16_t gx = (data[8] << 8) | data[9];
        int16_t gy = (data[10] << 8) | data[11];
        int16_t gz = (data[12] << 8) | data[13];
        //  printf("Accel X:%d Y:%d Z:%d | Gyro X:%d Y:%d Z:%d\n", ax, ay, az, gx, gy, gz);

        float ax_g = ax / 16384.0;
        float ay_g = ay / 16384.0;
        float az_g = az / 16384.0;

        float gx_dps = gx / 131.0;
        float gy_dps = gy / 131.0;
        float gz_dps = gz / 131.0;

        printf("Accel => X:%6.2f g | Y:%6.2f g | Z:%6.2f g || Gyro => X:%7.2f °/s | Y:%7.2f °/s | Z:%7.2f °/s\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
        // Mostra no monitor serial
        sleep_ms(1000);
    }
}
