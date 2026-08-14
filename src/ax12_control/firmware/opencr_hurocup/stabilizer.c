/* firmware/src/stabilizer.c — porte 1:1 do original.
 * Ankle-strategy postural stabilizer.
 * Matemática mantida IDÊNTICA ao original (ver nota em stabilizer.h
 * sobre a FPU disponível na OpenCR não ter sido explorada aqui).
 */
#include "stabilizer.h"

void stabilizer_init(stabilizer_state_t *state)
{
    state->prev_pitch_error = 0.0f;
    state->prev_roll_error  = 0.0f;
}

void stabilizer_set_defaults(stabilizer_params_t *params)
{
    params->kp_pitch       = 0.3f;
    params->kd_pitch       = 0.01f;
    params->kp_roll        = 0.2f;
    params->kd_roll        = 0.01f;
    params->max_correction = 0.15f;
    params->pitch_offset   = 0.0f;  /* overwritten by trunk_pitch before first use */
    params->roll_offset    = 0.0f;
}

/* Inline helper: clamp f to [-limit, +limit]. */
static float clamp(float f, float limit)
{
    if (f >  limit) { return  limit; }
    if (f < -limit) { return -limit; }
    return f;
}

void stabilizer_compute(
    const stabilizer_params_t *params,
    stabilizer_state_t        *state,
    float measured_pitch,
    float measured_roll,
    float pitch_rate,
    float roll_rate,
    float dt,
    stabilizer_output_t *output)
{
    (void)dt;  /* fixed 10 ms loop — dt not needed for direct gyro D term */

    float pitch_error = params->pitch_offset - measured_pitch;
    float roll_error  = params->roll_offset  - measured_roll;

    /* PD: P on error, D on -gyro_rate (damps without differentiating error). */
    float pitch_corr = params->kp_pitch * pitch_error
                     + params->kd_pitch * (-pitch_rate);
    float roll_corr  = params->kp_roll  * roll_error
                     + params->kd_roll  * (-roll_rate);

    pitch_corr = clamp(pitch_corr, params->max_correction);
    roll_corr  = clamp(roll_corr,  params->max_correction);

    /* Store errors for potential future use (e.g. integral term). */
    state->prev_pitch_error = pitch_error;
    state->prev_roll_error  = roll_error;

    /* Same correction applied symmetrically to both ankles.
     * Sign inversion per servo is handled in the main sketch (LEG_SERVO_SIGN). */
    output->r_ankle_pitch_corr = pitch_corr;
    output->l_ankle_pitch_corr = pitch_corr;
    output->r_ankle_roll_corr  = roll_corr;
    output->l_ankle_roll_corr  = roll_corr;
}
