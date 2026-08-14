/* firmware/src/stabilizer.h — porte 1:1 do original.
 * Ankle-strategy postural stabilizer using IMU pitch/roll feedback.
 * Runs at 100 Hz; adds PD corrections to the ankle joints.
 *
 * OPENCR: o comentário original ("no FPU, só add/mul") não se aplica
 * mais — o Cortex-M7 da OpenCR tem FPU de hardware. A matemática abaixo
 * foi mantida IDÊNTICA de propósito (mesmo ganho/comportamento
 * validado); usar a FPU disponível pra enriquecer o controlador (termo
 * integral, trig de verdade) é uma melhoria em aberto, não feita aqui.
 */
#ifndef STABILIZER_H
#define STABILIZER_H

typedef struct {
    float kp_pitch;       /* proportional gain, pitch  (default 0.3)  */
    float kd_pitch;       /* derivative gain,   pitch  (default 0.01) */
    float kp_roll;        /* proportional gain, roll   (default 0.2)  */
    float kd_roll;        /* derivative gain,   roll   (default 0.01) */
    float max_correction; /* rad, saturation limit     (default 0.15) */
    float pitch_offset;   /* rad, desired pitch offset (referencia externa) */
    float roll_offset;    /* rad, desired roll  (normally 0)          */
} stabilizer_params_t;

typedef struct {
    float prev_pitch_error;
    float prev_roll_error;
} stabilizer_state_t;

/* Corrections to apply at the four ankle DoF (radians). */
typedef struct {
    float r_ankle_pitch_corr;
    float r_ankle_roll_corr;
    float l_ankle_pitch_corr;
    float l_ankle_roll_corr;
} stabilizer_output_t;

void stabilizer_init(stabilizer_state_t *state);
void stabilizer_set_defaults(stabilizer_params_t *params);

/*
 * stabilizer_compute — call every tick (dt = 0.01 s).
 * pitch_rate / roll_rate come directly from the IMU gyro (rad/s).
 * D term uses -gyro_rate to damp oscillation without differentiating
 * the noisy error signal.
 */
void stabilizer_compute(
    const stabilizer_params_t *params,
    stabilizer_state_t        *state,
    float measured_pitch,   /* IMU complementary-filter output (rad) */
    float measured_roll,    /* IMU complementary-filter output (rad) */
    float pitch_rate,       /* gyro_y (rad/s)                        */
    float roll_rate,        /* gyro_x (rad/s)                        */
    float dt,
    stabilizer_output_t *output
);

#endif /* STABILIZER_H */
