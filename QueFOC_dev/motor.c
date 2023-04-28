//
// Created by 24216 on 2022/12/23.
//

#include "motor.h"
#include "foc.h"
#include <string.h>

void motor_init(Motor* motor,
                uint8_t id, uint32_t KV, float calib_current, float calib_voltage,
                float ctrl_i_bandwidth, float inertia,
                float limit_i_bus_max, float limit_i_leak_max, float limit_v_bus_min, float limit_v_bus_max,
                float limit_vel_max, float limit_i_max,
                Encoder* encoder, ADC_sensor* adc, PWM* pwm, Cmder* cmder,
                void (*hardware_basic_boot)(void)){
    // clean up
    memset(motor, 0, sizeof(Motor));

    motor->id = id;
    motor->calib_current = calib_current;
    motor->calib_voltage = calib_voltage;
    motor->ctrl_i_bandwidth = ctrl_i_bandwidth;
    motor->inertia = inertia;

    motor->limit_i_bus_max = limit_i_bus_max;
    motor->limit_i_leak_max = limit_i_leak_max;
    motor->limit_v_bus_min = limit_v_bus_min;
    motor->limit_v_bus_max = limit_v_bus_max;
    motor->limit_vel_max = limit_vel_max;
    motor->limit_i_max = limit_i_max;


    motor->encoder = encoder;
    motor->adc = adc;
    motor->pwm = pwm;
    motor->cmder = cmder;
    motor->hardware_basic_boot = hardware_basic_boot;

    // A approximation for torque constant (Nm/A) from Odrive
    motor->torque_constant = 8.27f / (float)(KV);

    motor->state = MS_CREATED;
}

void motor_boot(Motor* motor){
    motor->hardware_basic_boot();
    // FIXME
    encoder_boot(motor->encoder);
    adc_boot(motor->adc);
    pwm_boot(motor->pwm);

    motor->state = MS_BOOTED;
}

void motor_stop(Motor* motor){
    foc_refresh(motor);
    pwm_apply_duty(motor->pwm, 0, 0, 0);
    // FIXME
    motor->state = MS_IDLE;
}

void motor_shift_state(Motor* motor, Motor_state state){
    motor->state = state;
}

void interrupt_adc_done(Motor* motor){
    motor_loop_main(motor);
}

void motor_loop_main(Motor* motor){
    adc_update(motor->adc);
    // FIXME
    //encoder_update(motor->encoder);
    if(IF_CONTAIN_IN(motor->state, MS_RUN_BEGIN, MS_RUN_END))
        motor_loop_run(motor);
    else if(IF_CONTAIN_IN(motor->state, MS_CALIBRATION_BEGIN, MS_CALIBRATION_END))
        motor_loop_calibration(motor);
    else if(IF_CONTAIN_IN(motor->state, MS_ERROR_BEGIN, MS_ERROR_END))
        motor_loop_error(motor);
    else if(motor->state == MS_BOOTED)
        motor->state = MS_CALIBRATION_BEGIN;
    else if(motor->state == MS_CALIBRATION_END)
        motor->state = MS_IDLE;
    else if(motor->state == MS_IDLE || motor->state == MS_CREATED)
        return;
    else
        motor->state = MS_ERROR_UNKNOWN_STATE;
}

void motor_loop_calibration(Motor* motor){
    static uint32_t step_loop_cnt = 0;
    static float voltages[3] = {0};
    static float currents_f[3] = {0};
    static uint32_t currents[3] = {0};
    uint32_t phase_resistance_loop_cnt = 3*motor->adc->measure_freq;
    uint32_t phase_inductance_loop_cnt = motor->adc->measure_freq/2;
    float cc_voltage = motor->calib_current * motor->phase_resistance;
    static float cur_phase = 0;

    // current step loop cnt
    step_loop_cnt += 1;

    switch (motor->state) {
        case MS_CALIBRATION_BEGIN:
        {
            motor->state = MS_CALIBRATION_PHASE_CURRENT_OFFSET_INIT;
        }
            break;
        /* phase current OP-AMP offset  */
        case MS_CALIBRATION_PHASE_CURRENT_OFFSET_INIT:
        {
            // set phase current to zero
            pwm_apply_duty(motor->pwm, 0, 0, 0);
            step_loop_cnt = 0;
            currents[0] = currents[1] = currents[2] = 0;
            motor->state = MS_CALIBRATION_PHASE_CURRENT_OFFSET_LOOP;

        }
            break;
        case MS_CALIBRATION_PHASE_CURRENT_OFFSET_LOOP:
        {
            currents[0] += motor->adc->hardware_adc_read_phase_a();
            currents[1] += motor->adc->hardware_adc_read_phase_b();
            currents[2] += motor->adc->hardware_adc_read_phase_c();
            if(step_loop_cnt >= 64){
                motor->adc->i_op_amp_offset_a = currents[0] / step_loop_cnt;
                motor->adc->i_op_amp_offset_b = currents[1] / step_loop_cnt;
                motor->adc->i_op_amp_offset_c = currents[2] / step_loop_cnt;
                uint16_t max_error_offset = (uint16_t)(0.1/3.3f*motor->adc->resolution);
                if(IF_CONTAIN_IN2(motor->adc->i_op_amp_offset_a, motor->adc->i_op_amp_offset_theory, max_error_offset) &&
                   IF_CONTAIN_IN2(motor->adc->i_op_amp_offset_b, motor->adc->i_op_amp_offset_theory, max_error_offset) &&
                   IF_CONTAIN_IN2(motor->adc->i_op_amp_offset_c, motor->adc->i_op_amp_offset_theory, max_error_offset) )
                    motor->state = MS_CALIBRATION_MEASURE_PHASE_RESISTANCE_INIT;
                else{
                    // Self-check fail
                    motor->state = MS_ERROR_CALIBRATION_PHASE_CURRENT_OFFSET;
                    cmder_report_prt(motor->cmder, "I_offset_a=%d\nI_offset_b=%d\nI_offset_c=%d\nI_offset_theory=%d\nmax_error_offset=%d\n",
                                    motor->adc->i_op_amp_offset_a, motor->adc->i_op_amp_offset_b, motor->adc->i_op_amp_offset_c,
                                    motor->adc->i_op_amp_offset_theory, max_error_offset);
                }

            }
        }
            break;

        case MS_CALIBRATION_MEASURE_PHASE_RESISTANCE_INIT:
        {
            step_loop_cnt = 0;
            voltages[0] = 0;
            motor->state = MS_CALIBRATION_MEASURE_PHASE_RESISTANCE_LOOP;
        }
            break;
        case MS_CALIBRATION_MEASURE_PHASE_RESISTANCE_LOOP:
        {
            // Integral controller
            float ki = 2;
            voltages[0] += ki * motor->adc->measure_period * (motor->calib_current - motor->adc->i_a);
            if(ABS(voltages[0]) > 5){
                pwm_apply_duty(motor->pwm, 0, 0, 0);
                motor->state = MS_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
                //cmder_report_prt(motor->cmder, "%f %f %f\nV=%f\n", motor->adc->i_a, motor->adc->i_b, motor->adc->i_c, voltages[0]);
                break;
            }
            // Phase A
            if(foc_voltage_control(motor, voltages[0], 0, 0))
                pwm_sync_duty(motor->pwm);
            /*
            if(step_loop_cnt % 100 == 0){
                cmder_report_prt(motor->cmder, "%.3f %.3f %.3f\nV=%f\n", motor->adc->i_a, motor->adc->i_b, motor->adc->i_c, voltages[0]);
            }
            */
            if(step_loop_cnt >= phase_resistance_loop_cnt){

                motor->phase_resistance = voltages[0]/motor->calib_current *2.0f/3.0f;
                //cmder_report_prt(motor->cmder, "%f\n", voltages[0]);
                //cmder_report_prt(motor->cmder, "%f\nphase_resistance=%f\n", motor->adc->i_a, motor->phase_resistance);
                motor->state = MS_CALIBRATION_MEASURE_PHASE_INDUCTANCE_INIT;
                pwm_apply_duty(motor->pwm, 0, 0, 0);
            }
        }
            break;
        case MS_CALIBRATION_MEASURE_PHASE_INDUCTANCE_INIT:
        {
            step_loop_cnt = 0;
            voltages[0] = -motor->calib_voltage;
            voltages[1] = +motor->calib_voltage;
            currents_f[0] = currents_f[1] = 0;
            if(foc_voltage_control(motor, voltages[0], 0, 0))
                pwm_sync_duty(motor->pwm);
            motor->state = MS_CALIBRATION_MEASURE_PHASE_INDUCTANCE_LOOP;
        }
            break;
        case MS_CALIBRATION_MEASURE_PHASE_INDUCTANCE_LOOP:
        {
            uint32_t idx = step_loop_cnt & 1;
            currents_f[idx] += motor->adc->i_a;

            // Phase A
            if(foc_voltage_control(motor, voltages[idx], 0, 0))
                pwm_sync_duty(motor->pwm);
            if(step_loop_cnt >= phase_inductance_loop_cnt*2){
                //cmder_report_prt(motor->cmder, "%.3f,%.3f\n", currents_f[0], currents_f[1]);
                float dI_by_dt = (currents_f[1] - currents_f[0]) / (float)(phase_inductance_loop_cnt * motor->adc->measure_period);
                motor->phase_inductance = motor->calib_voltage / dI_by_dt *2.0f/3.0f;
                //cmder_report_prt(motor->cmder, "phase_inductance=%f\n", motor->phase_inductance);
                // Update current ctrl param
                motor_update_current_ctrl_param(motor);

                pwm_apply_duty(motor->pwm, 0, 0, 0);
                // FIXME
                motor->state = MS_CALIBRATION_END;
            }
        }
            break;
        case MS_CALIBRATION_CHECK_DIRECTION_AND_POLE_PAIRS_INIT:
        {
            step_loop_cnt = 0;
            cur_phase = 0;
        }
            break;
        case MS_CALIBRATION_CHECK_DIRECTION_AND_POLE_PAIRS_LOOP:
        {
            cur_phase += PI * motor->adc->measure_period;
            if(foc_voltage_control(motor, cc_voltage, 0, cur_phase))
                pwm_sync_duty(motor->pwm);
            if(cur_phase >= 2*PIx2){

            }
        }
            break;
        case MS_CALIBRATION_END:
        {
            cmder_report_prt(motor->cmder, "Motor %c calibration success\n", (char)motor->id);
            cmder_report_prt(motor->cmder, "torque_constant=%f\n", motor->torque_constant);
            cmder_report_prt(motor->cmder, "I_offset_a=%d\nI_offset_b=%d\nI_offset_c=%d\nI_offset_theory=%d\n",
                                motor->adc->i_op_amp_offset_a, motor->adc->i_op_amp_offset_b, motor->adc->i_op_amp_offset_c,
                                motor->adc->i_op_amp_offset_theory);
            cmder_report_prt(motor->cmder, "phase_resistance=%f(%fA)\n", motor->phase_resistance, motor->calib_current);
            cmder_report_prt(motor->cmder, "phase_inductance=%f(%fV)\n", motor->phase_inductance, motor->calib_voltage);
            motor->state = MS_IDLE;
        }
            break;
        default:
            break;
    }

}

void motor_loop_run(Motor* motor){
    switch (motor->state) {
        case MS_RUN_TORQUE:
        {
            motor->torque_sp = motor->torque_input;
        }
            break;
        case MS_RUN_VELOCITY:
        {
            float delta_vel = motor->vel_input - motor->vel_sp;
            // T = a * J
            motor->torque_sp = (delta_vel / motor->adc->measure_period) * motor->inertia;
            motor->vel_sp += delta_vel;
        }
            break;
        default:
            break;
    }

    float torque = motor->torque_sp;
    float vel = motor->vel_sp;
    float max_torque = motor->torque_constant * motor->limit_i_max;
    bool torque_limited = false;
    if(torque > max_torque){
        torque = max_torque;
        torque_limited = true;
    }else if(torque < -max_torque){
        torque = -max_torque;
        torque_limited = true;
    }

    float iq_set = torque / motor->torque_constant;
    float I_phase = motor->encoder->phase;
    // 1.5 T_measure delay ?
    float PWM_phase = I_phase + 1.5f*motor->adc->measure_period*motor->encoder->phase_vel;
    if(foc_current_control(motor, 0, iq_set, I_phase, PWM_phase))
        pwm_sync_duty(motor->pwm);
}

void motor_loop_error(Motor* motor){
    switch (motor->state) {
        case MS_ERROR_CALIBRATION_PHASE_CURRENT_OFFSET:
        {
            cmder_report_error(motor->cmder, motor, "CALIBRATION PHASE CURRENT OFFSET SELF CHECK FAIL");
        }
            break;
        case MS_ERROR_SVM_RESULT_INVALID:
        {
            cmder_report_error(motor->cmder, motor, "SVM RESULT INVALID");
        }
            break;
        case MS_ERROR_UNKNOWN_STATE:
        {
            cmder_report_error(motor->cmder, motor, "UNKNOWN MOTOR STATE");
        }
            break;
        case MS_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE:
        {
            cmder_report_error(motor->cmder, motor, "PHASE RESISTANCE OUT OF RANGE");
        } break;
        default:
        {
            cmder_report_error(motor->cmder, motor, "UNKNOWN ERROR");
        }
            break;
    }
    motor_stop(motor);
    //while(1){}
}
void motor_update_current_ctrl_param(Motor* motor){
    motor->ctrl_i_kp = motor->phase_inductance * motor->ctrl_i_bandwidth;
    float plant_pole = motor->phase_resistance / motor->phase_inductance;
    motor->ctrl_i_ki = plant_pole * motor->ctrl_i_kp;
}
