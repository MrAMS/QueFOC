/*
 * hardware.h
 *
 *  Created on: Dec 31, 2022
 *      Author: 24216
 */

#ifndef HARDWARE_HARDWARE_H_
#define HARDWARE_HARDWARE_H_

#include "ch32v30x.h"

/* SYSTEM */
#define PWM_FREQUENCY               20000
/* Motor */
#define KV                          980
#define INERTIA                     0
/* Driver */
#define ADC_RESOLUTION_BIT          12
#define V_BUS_GAIN                  19
#define SHUNT_RESISTANCE            0.03f
#define I_OP_AMP_OFFSET             1.65f
#define I_OP_AMP_GAIN               5.0f
#define CALIB_CURRENT               3.0f
#define CALIB_VOLTAGE               3.0f


/* ADC_CLK = PCLK2 / ADC_CLK_DIV
 * Note that CH32V30x ADCCLLK < 14 MHz
 * FIXME SYSCLK=72MHz, 72/6 = 12
 */
#define ADC_CLK_DIV RCC_PCLK2_Div6
// FIXME
// SAMPLE_TIME_CLK + 12.5 * ADC_CLK
// 14 * (1/12)MHz = 1.16us
#define ADC_SAMPLE_TIME ADC_SampleTime_1Cycles5

#define TIMER1_CLK                  SystemCoreClock
#define PWM_PERIOD_CYCLES           (uint16_t)((TIMER1_CLK/((uint32_t)(PWM_FREQUENCY)))&0xFFFE)
#define HALF_PWM_PERIOD_CYCLES      (uint16_t)(PWM_PERIOD_CYCLES/2U)
// FIXME
// IR2136 290ns
#define PWM_DEADTIME                21


/* GPIO */
void hardware_gpio_boot();

/* NVIC */
void hardware_nvic_boot();

/* DMA */
void hardware_dma_boot();

/* ADC */
void hardware_adc_boot();
u16 hardware_adc_read_phase_a();
u16 hardware_adc_read_phase_b();
u16 hardware_adc_read_phase_c();
u16 hardware_adc_read_vbus(void);
u16 hardware_adc_read_vgnd(void);

/* PWM */
void hardware_pwm_boot(void);
void hardware_set_pwm_duty_ch_a(float duty);
void hardware_set_pwm_duty_ch_b(float duty);
void hardware_set_pwm_duty_ch_c(float duty);

/* USART */
void hardware_usart3_boot(void);
void hardware_usart3_send(const char* data);


#endif /* HARDWARE_HARDWARE_H_ */
