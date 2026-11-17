// On/off and PID controllers. Pure C, output in percent duty, unit-tested on the host.
#pragma once
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------- on/off with hysteresis
typedef struct {
    float hyst_c;
    float on_pct;   // duty while on (usually 100)
    bool on;
} onoff_t;

void onoff_init(onoff_t *c, float hyst_c, float on_pct);
float onoff_update(onoff_t *c, float setpoint_c, float temp_c);

// ---------------------------------------------------------------- PID
// u = Kp*e + integral + Kd*d(-T)/dt
// - derivative on measurement, so a setpoint step does not kick the output
// - first-order filter on the derivative (alpha in 0..1, 1 = unfiltered)
// - anti-windup by conditional integration: the integral only moves when the
//   output is not saturated, or when the error would pull it out of saturation
typedef struct {
    float kp, ki, kd;
    float d_alpha;
    float out_min, out_max;
    float integral;     // in output units (%)
    float d_state;      // filtered derivative of the measurement (C/s)
    float prev_temp;
    bool primed;        // false until the first sample
    // last terms, for logging/debugging
    float p_term, i_term, d_term;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *c, float kp, float ki, float kd, float d_alpha, float out_min, float out_max);
void pid_reset(pid_ctrl_t *c);
float pid_update(pid_ctrl_t *c, float setpoint_c, float temp_c, float dt_s);

#ifdef __cplusplus
}
#endif
