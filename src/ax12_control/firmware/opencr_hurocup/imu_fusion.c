/* Porte 1:1 do original — matemática pura, nenhuma mudança de fórmula.
 * VER O AVISO em imu_fusion.h: o remapeamento de eixos abaixo foi
 * derivado da montagem física do BMI160 na placa antiga e PRECISA ser
 * reconferido com o IMU real da OpenCR (outra montagem, outro chip). */
#include <math.h>
#include "imu_fusion.h"

/* Complementary-filter blend factor (gyro weight per step at ~100 Hz). */
#define IMU_ALPHA   0.98f

/*
 * Physical mount (measured on the bench, projeto original, 2026-07-11):
 *   Standing upright the BMI160 reads accel ~= (-9.67, +0.50, -0.14) m/s²,
 *   i.e. GRAVITY RESTS ON THE SENSOR -X AXIS, not on +Z.
 *
 * Sensor -> REP-103 body frame (X forward, Y left, Z up), verified by
 * tilt tests on the ORIGINAL board (forward tilt -> pitch rises; right
 * tilt -> roll rises):
 *   up      (+Z_body) = -accel_x
 *   left    (+Y_body) = -accel_y
 *   forward (+X_body) = -accel_z
 *   pitch-rate (about Y) = -gyro_y     roll-rate (about X) = +gyro_z
 *   yaw-rate   (about Z) = +gyro_x
 *
 * Field roles (legacy names in imu_state_t are scrambled; mantido igual
 * ao original de propósito — não renomear sem atualizar o consumidor
 * ROS/ax12_control que você for escrever depois):
 *   state->pitch = geometric pitch   (+ = forward / nose down)
 *   state->yaw   = geometric ROLL     (+ = leaning right)
 *   state->roll  = geometric YAW / heading (integrated about up; drifts, no mag)
 *
 * Tilt angles from accel use the standard AN3461 decomposition so pitch and
 * roll stay decoupled when both are non-zero.
 */
void imu_fuse(imu_state_t *state, float dt)
{
    /* Remap sensor axes to the body frame (see header comment). */
    const float a_up   = -state->accel_x;   /* +Z body (up)      */
    const float a_left = -state->accel_y;   /* +Y body (left)    */
    const float a_fwd  = -state->accel_z;   /* +X body (forward) */

    /* Accelerometer tilt (AN3461): pitch decoupled from roll. */
    const float pitch_acc = atan2f(-a_fwd, sqrtf(a_left * a_left + a_up * a_up));
    const float roll_acc  = atan2f(a_left, a_up);

    /* pitch: gyro rate about body Y is -gyro_y (forward = positive). */
    state->pitch = IMU_ALPHA * (state->pitch - state->gyro_y * dt)
                 + (1.0f - IMU_ALPHA) * pitch_acc;

    /* lateral roll (stored in the legacy 'yaw' field): rate about body X is +gyro_z. */
    state->yaw = IMU_ALPHA * (state->yaw + state->gyro_z * dt)
               + (1.0f - IMU_ALPHA) * roll_acc;

    /* heading (legacy 'roll' field): integrate yaw-rate about body up axis (+gyro_x).
     * No accelerometer reference for heading -> drifts without a magnetometer. */
    state->roll += state->gyro_x * dt;
}
