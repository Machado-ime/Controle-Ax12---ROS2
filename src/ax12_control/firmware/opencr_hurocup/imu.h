#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/*
 * imu_state_t — all IMU data in SI units. Interface IDÊNTICA ao
 * original (mesmos nomes de campo, mesma convenção "scrambled" —
 * documentada abaixo — pra não quebrar quem for escrever o consumidor
 * ROS depois). Fonte física: OpenCR (MPU9250 ou ICM-20648 via lib
 * oficial cIMU), não mais o BMI160 do projeto original.
 *
 * accel_*  : m/s²
 * gyro_*   : rad/s
 * pitch, roll, yaw : radians (complementary filter, ver imu_fusion.c).
 *   Convenção herdada do original (NÃO é a ordem geométrica óbvia):
 *   pitch = geometric pitch  (+ = forward / nose down)
 *   yaw   = geometric ROLL    (+ = leaning right)
 *   roll  = geometric YAW / heading (integra giro sobre o eixo up; drifta, sem mag)
 * gyro_offset_* : rad/s offsets medidos em imu_calibrate()
 */
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float pitch,   roll,    yaw;
    float gyro_offset_x, gyro_offset_y, gyro_offset_z;
    uint32_t timestamp_ms;
    uint8_t  initialized;
} imu_state_t;

#ifdef __cplusplus
extern "C" {
#endif

/*
 * imu_init() — inicializa o sensor via lib oficial cIMU (IMU.begin()).
 * OPENCR: substitui o soft-reset/wake manual do BMI160 por I2C.
 */
void imu_init(void);

/*
 * imu_bus_recover() — OPENCR: no-op. O original bit-bangava SCL 9x pra
 * destravar um I2C1 preso; a lib cIMU não expõe um equivalente público,
 * e o barramento do sensor da OpenCR não é um I2C genérico acessível
 * pelo sketch. Mantida a assinatura pra não obrigar mudar o .ino se um
 * dia precisar de conteúdo real aqui.
 */
void imu_bus_recover(void);

/*
 * imu_who_am_i() — OPENCR: NÃO é uma leitura literal de chip-ID (a
 * lib cIMU não expõe esse registrador no nível do sketch, ao contrário
 * do bmi_read(BMI_CHIP_ID) do original). Faz uma checagem de
 * plausibilidade (magnitude do vetor de aceleração ~= 1 g) como smoke
 * test de "o sensor está respondendo com dado sensato". Retorna 0xD1
 * (mesmo valor "saudável" do original) se plausível, 0x00 caso não.
 */
uint8_t imu_who_am_i(void);

/*
 * imu_calibrate() — porte 1:1 do algoritmo original: médias de
 * `samples` leituras cruas de giro com o robô parado, guarda em
 * gyro_offset_* (rad/s). Bloqueia por samples x 10 ms.
 */
void imu_calibrate(imu_state_t *state, uint16_t samples);

/*
 * imu_read_and_filter() — lê o sensor, converte pra SI, aplica o
 * filtro complementar (imu_fuse(), inalterado). dt = segundos desde a
 * chamada anterior (nominal 0.01 a 100 Hz).
 *
 * CONFIRME NO HARDWARE (ver aviso em imu_fusion.h): a montagem física
 * do IMU na OpenCR é diferente da placa original — repita os testes de
 * tombar o robô antes de confiar no sinal de pitch/roll.
 */
void imu_read_and_filter(imu_state_t *state, float dt);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
