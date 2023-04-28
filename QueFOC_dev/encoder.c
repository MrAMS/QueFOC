//
// Created by 24216 on 2022/12/21.
//

#include "encoder.h"

void encoder_init(Encoder *encoder, uint32_t freq, int motor_pole_pairs, void (*hardware_boot)(void), bool (*hardware_get_abs_pos_cnt)(uint16_t*)){
    encoder->measure_period = freq;
    encoder->motor_pole_pairs = motor_pole_pairs;
    encoder->hardware_boot = hardware_boot;
    encoder->hardware_get_abs_pos_cnt = hardware_get_abs_pos_cnt;
}

void encoder_boot(Encoder *encoder){
    encoder->hardware_boot();
}

static inline int std_delta_cnt(Encoder *encoder, int delta_cnt){
    int res = mod_real(delta_cnt, encoder->cpr);
    if(res > encoder->cpr/2)
        res -= encoder->cpr;
    return res;
}

void encoder_update(Encoder *encoder){

    uint16_t raw = 0;
    if(!encoder->hardware_get_abs_pos_cnt(&raw)){
        encoder->cnt_rx_error += 1;
        return;
    }

    if(encoder->dir > 0)
        encoder->cnt_raw = raw;
    else
        encoder->cnt_raw = encoder->cpr - raw;

    int cnt_delta = std_delta_cnt(encoder, encoder->cnt_raw - encoder->cnt_raw_pre);
    // note that cnt_sum may overflow
    encoder->cnt_sum += cnt_delta;
    // update cnt_raw_pre
    encoder->cnt_raw_pre = encoder->cnt_raw;

    /* PLL algorithm to estimate velocity */

    // estimate position
    encoder->cnt_pos_estimate += encoder->measure_period * encoder->cnt_vel_estimate;
    // calc estimate error
    int cnt_pos_estimate_error = std_delta_cnt(encoder, encoder->cnt_raw - (int)(encoder->cnt_pos_estimate));

    encoder->cnt_pos_estimate += encoder->measure_period * encoder->pll_kp * (float)cnt_pos_estimate_error;
    encoder->cnt_pos_estimate = fmodf_real(encoder->cnt_pos_estimate, (float)encoder->cpr);
    encoder->cnt_vel_estimate += encoder->measure_period * encoder->pll_ki * (float)cnt_pos_estimate_error;

    encoder->vel = encoder->cnt_vel_estimate / (float)encoder->cpr;
    encoder->pos = (float)(encoder->cnt_sum/encoder->cpr) + (float)(encoder->cnt_sum%encoder->cpr)/(float)encoder->cpr;

    bool snap_to_zero_vel = false;
    if (ABS(encoder->cnt_vel_estimate) < 0.5f * encoder->measure_period * encoder->pll_ki) {
        encoder->cnt_vel_estimate = 0.0f;  //align delta-sigma on zero to prevent jitter
        snap_to_zero_vel = true;
    }

    /* Encoder count interpolation */

    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        encoder->interpolation = 0.5f;
    // reset interpolation if encoder edge comes
    } else if (cnt_delta > 0) {
        encoder->interpolation = 0.0f;
    } else if (cnt_delta < 0) {
        encoder->interpolation = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        encoder->interpolation += encoder->measure_period * encoder->cnt_vel_estimate;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        encoder->interpolation = CONTAIN_IN(encoder->interpolation, 0, 1);
    }
    float real_interpolated_pos_cnt = (float)(encoder->cnt_raw - encoder->offset) + encoder->interpolation;
    real_interpolated_pos_cnt = fmodf_real(real_interpolated_pos_cnt, (float)encoder->cpr);

    encoder->phase = (real_interpolated_pos_cnt * PIx2 * (float)encoder->motor_pole_pairs)/(float)encoder->cpr;
    encoder->phase_vel = encoder->vel * PIx2 * (float)encoder->motor_pole_pairs;
}
