/* OPENCR: reescrito. O original acessava o BMI160 por I2C manual
 * (bmi_write/bmi_read com timeout, ~180 linhas de RM0008 §I2C). Aqui a
 * lib oficial cIMU (<IMU.h>, mesma usada no opencr_op3 da ROBOTIS)
 * cuida do sensor físico (MPU9250 ou ICM-20648, dependendo da revisão
 * da placa) — a interface pública (imu.h) fica idêntica ao original,
 * só o conteúdo interno muda. Arquivo .cpp (não .c) porque cIMU é uma
 * classe C++; os wrappers abaixo são extern "C" para o resto do
 * firmware (100% C) chamar sem mudança.
 */
#include <Arduino.h>
#include <math.h>
#include <IMU.h>
#include "imu.h"
#include "imu_fusion.h"

static cIMU IMU;

/* Fatores de conversão — os MESMOS usados no open_cr_module oficial do
 * OP3 (ROBOTIS-OP3/open_cr_module), confirmados na tabela de registro
 * do firmware opencr_op3: raw int16 -> unidade física. */
#define GYRO_GRAUS_S_POR_LSB   (2000.0f / 32800.0f)
#define ACC_G_POR_LSB          (2.0f / 32768.0f)
#define G_PARA_M_S2            9.80665f

extern "C" void imu_init(void)
{
    IMU.begin();
}

extern "C" void imu_bus_recover(void)
{
    /* No-op — ver comentário em imu.h. */
}

extern "C" uint8_t imu_who_am_i(void)
{
    /* Smoke test de plausibilidade (não é WHO_AM_I literal — ver imu.h):
     * módulo do vetor de aceleração deve ficar perto de 1 g com o robô
     * parado em qualquer orientação. */
    IMU.update();
    float ax = (float)IMU.SEN.accRAW[0] * ACC_G_POR_LSB;
    float ay = (float)IMU.SEN.accRAW[1] * ACC_G_POR_LSB;
    float az = (float)IMU.SEN.accRAW[2] * ACC_G_POR_LSB;
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    return (mag > 0.7f && mag < 1.3f) ? 0xD1U : 0x00U;
}

extern "C" void imu_calibrate(imu_state_t *state, uint16_t samples)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (uint16_t i = 0U; i < samples; i++) {
        IMU.update();
        sum_x += (float)IMU.SEN.gyroADC[0];
        sum_y += (float)IMU.SEN.gyroADC[1];
        sum_z += (float)IMU.SEN.gyroADC[2];
        delay(10);
    }

    float n = (float)samples;
    float scale = GYRO_GRAUS_S_POR_LSB * (float)M_PI / 180.0f;  /* LSB -> rad/s */
    state->gyro_offset_x = (sum_x / n) * scale;
    state->gyro_offset_y = (sum_y / n) * scale;
    state->gyro_offset_z = (sum_z / n) * scale;
    state->initialized   = 1U;
}

extern "C" void imu_read_and_filter(imu_state_t *state, float dt)
{
    IMU.update();

    float scale_g  = GYRO_GRAUS_S_POR_LSB * (float)M_PI / 180.0f;  /* LSB -> rad/s */
    float scale_a  = ACC_G_POR_LSB * G_PARA_M_S2;                  /* LSB -> m/s²  */

    float gx = (float)IMU.SEN.gyroADC[0] * scale_g;
    float gy = (float)IMU.SEN.gyroADC[1] * scale_g;
    float gz = (float)IMU.SEN.gyroADC[2] * scale_g;
    float ax = (float)IMU.SEN.accRAW[0]  * scale_a;
    float ay = (float)IMU.SEN.accRAW[1]  * scale_a;
    float az = (float)IMU.SEN.accRAW[2]  * scale_a;

    /* ATENÇÃO: eixo/sinal abaixo copiado do original (mesma convenção de
     * variável), mas a montagem física do sensor na OpenCR É DIFERENTE
     * da placa original — reconfirme no hardware (ver imu_fusion.h)
     * antes de considerar esses sinais corretos. */
    state->accel_x =  ax;
    state->accel_y = -ay;
    state->accel_z =  az;

    state->gyro_x = -(gx - state->gyro_offset_x);
    state->gyro_y =   gy - state->gyro_offset_y;
    state->gyro_z = -(gz - state->gyro_offset_z);

    imu_fuse(state, dt);

    state->timestamp_ms = millis();
}
