#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include "imu.h"

/*
 * imu_fuse() — porte 1:1 do original: filtro complementar puro (sem
 * acesso a hardware).
 *
 * Consumes state->accel_* and state->gyro_* (SI units, gyro already
 * bias-corrected by imu_read_and_filter) and updates state->pitch/roll/yaw.
 * dt is the elapsed time in seconds since the previous call.
 *
 * ATENÇÃO — precisa reverificar na OpenCR: o remapeamento de eixos
 * sensor->corpo abaixo (imu_fusion.c) foi calibrado para a montagem
 * FÍSICA do BMI160 na placa antiga (~90° girado, gravidade no -X). O
 * chip da OpenCR (MPU9250/ICM-20648) tem outra orientação de montagem
 * na sua própria placa — repita os 3 testes de bancada do comentário em
 * imu_fusion.c (robô em pé, tombar ~15° pra frente, tombar ~15° pra
 * direita) antes de confiar no sinal de pitch/roll.
 */
void imu_fuse(imu_state_t *state, float dt);

#endif /* IMU_FUSION_H */
