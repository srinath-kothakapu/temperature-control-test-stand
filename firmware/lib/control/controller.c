#include "controller.h"

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void onoff_init(onoff_t *c, float hyst_c, float on_pct)
{
    c->hyst_c = hyst_c;
    c->on_pct = on_pct;
    c->on = false;
}

float onoff_update(onoff_t *c, float setpoint_c, float temp_c)
{
    if (temp_c < setpoint_c - c->hyst_c) c->on = true;
    else if (temp_c > setpoint_c + c->hyst_c) c->on = false;
    // inside the band: keep the previous state
    return c->on ? c->on_pct : 0.0f;
}

void pid_init(pid_ctrl_t *c, float kp, float ki, float kd, float d_alpha, float out_min, float out_max)
{
    c->kp = kp;
    c->ki = ki;
    c->kd = kd;
    c->d_alpha = clampf(d_alpha, 0.0f, 1.0f);
    c->out_min = out_min;
    c->out_max = out_max;
    pid_reset(c);
}

void pid_reset(pid_ctrl_t *c)
{
    c->integral = 0.0f;
    c->d_state = 0.0f;
    c->prev_temp = 0.0f;
    c->primed = false;
    c->p_term = c->i_term = c->d_term = 0.0f;
}

float pid_update(pid_ctrl_t *c, float setpoint_c, float temp_c, float dt_s)
{
    float e = setpoint_c - temp_c;

    if (!c->primed || dt_s <= 0.0f) {
        c->prev_temp = temp_c;
        c->d_state = 0.0f;
        c->primed = true;
    } else {
        float dT = (temp_c - c->prev_temp) / dt_s;
        c->d_state += c->d_alpha * (dT - c->d_state);
        c->prev_temp = temp_c;
    }

    c->p_term = c->kp * e;
    c->d_term = -c->kd * c->d_state;

    float candidate = c->integral + c->ki * e * (dt_s > 0.0f ? dt_s : 0.0f);
    float u_unsat = c->p_term + candidate + c->d_term;

    // Conditional integration: accept the new integral unless it pushes further into saturation.
    bool high = u_unsat > c->out_max && e > 0.0f;
    bool low = u_unsat < c->out_min && e < 0.0f;
    if (!high && !low) c->integral = candidate;
    c->integral = clampf(c->integral, c->out_min, c->out_max);
    c->i_term = c->integral;

    return clampf(c->p_term + c->i_term + c->d_term, c->out_min, c->out_max);
}
