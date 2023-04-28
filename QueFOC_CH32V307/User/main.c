#include "motor.h"
#include "hardware.h"
#include "foc.h"

Motor motorQ={0};
Encoder encoderQ={0};
ADC_sensor adcQ={0};
PWM pwmQ={0};

Cmder cmder={0};
Monitor monitor={0};


void hardware_basic_boot(){
    hardware_motor_gpio_boot();
    hardware_nvic_boot();
    hardware_dma_boot();
}

void hardware_encoder_boot(){
    hardware_encoder_gpio_boot();
    hardware_encoder_spi_boot();
}

int main(void)
{
	//USART_Printf_Init(115200);
    hardware_delay_init();
#if USE_USB_CDC
    monitor_init(&monitor,
            hardware_usb_fs_cdc_boot, hardware_usb_cdc_send);
#else
    monitor_init(&monitor,
            hardware_usart3_boot, hardware_usart3_send);
#endif
    monitor_boot(&monitor);
    monitor_prt(&monitor, "SystemClk:%d\r\n", SystemCoreClock);

    encoder_init(&encoderQ, PWM_FREQUENCY, POLE_PAIRS, hardware_encoder_boot, hardware_encoder_spi_data);

	adc_sensor_init(&adcQ,
	        PWM_FREQUENCY, ADC_RESOLUTION_BIT,
	        I_OP_AMP_GAIN, I_OP_AMP_OFFSET, V_BUS_GAIN, SHUNT_RESISTANCE,
	        hardware_adc_boot,
	        hardware_adc_read_phase_a, hardware_adc_read_phase_b, hardware_adc_read_phase_c,
	        hardware_adc_read_vbus, hardware_adc_read_vgnd);
	pwm_init(&pwmQ,
	        PWM_FREQUENCY,
	        hardware_pwm_boot,
	        hardware_set_pwm_duty_ch_a, hardware_set_pwm_duty_ch_b, hardware_set_pwm_duty_ch_c);
	cmder_init(&cmder,
	        '\n', true,
	        hardware_usart3_boot, hardware_usart3_send);
	motor_init(&motorQ,
	        'Q', KV, CALIB_CURRENT, CALIB_VOLTAGE,
	        PWM_FREQUENCY*0.1, INERTIA,
	        2, 5, 12, 28,
	        100, I_MAX,
	        &encoderQ, &adcQ, &pwmQ, &cmder,
	        hardware_basic_boot);



	cmder_boot(&cmder);

	hardware_delay_ms(500);

	//encoder_boot(&encoderQ);

	motor_boot(&motorQ);

	cmder_add_motor(&cmder, &motorQ);

	//hardware_encoder_IIF_boot();


	monitor_prt(&monitor, "SystemClk:%d\r\n", SystemCoreClock);

	hardware_delay_ms(500);

	/* LED Blink */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE , ENABLE );
	GPIO_InitTypeDef GPIO_InitStructure={0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    //GPIO_ResetBits(GPIOB, GPIO_Pin_12);




	int led=0;
	while(1){
	    //cmder_report_info(&cmder, &motorQ, "LOOP\n");
	    led ^= 1;
	    if(led)
	        GPIO_ResetBits(GPIOE, GPIO_Pin_9);
	    else
	        GPIO_SetBits(GPIOE, GPIO_Pin_9);

	    /*
	    // FOC Debug current control
	    hardware_delay_ms(10);
        if(motorQ.state == MS_IDLE){
            monitor_prt(&monitor, "%.3f %.3f %.3f\n", motorQ.adc->i_a, motorQ.adc->i_b, motorQ.adc->i_c);
            extern float debug_id_set;
            if(foc_current_control(&motorQ, debug_id_set, 0, 0, 0))
                pwm_sync_duty(motorQ.pwm);
        }
        */

//        monitor_prt(&monitor, "FLAG\nTXE%d RXNE%d MODF%d BSY%d OVR%d\n", SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_I2S_FLAG_TXE),
//                                                                SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_I2S_FLAG_RXNE),
//                                                                SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_FLAG_MODF),
//                                                                SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_I2S_FLAG_BSY),
//                                                                SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_I2S_FLAG_OVR)
//                                                                );
	    //monitor_prt(&monitor, "%d\n", hardware_encoder_IIF_query());
	    u16 raw = 0;
	    //monitor_prt(&monitor, "tring write state=%d\n", SPI_I2S_GetFlagStatus(ENCODER_SPIx, SPI_I2S_FLAG_TXE));
	    u8 ret = encoderQ.hardware_get_abs_pos_cnt(&raw);
	    float angle = raw*360.0f/16384;
//	    u8 ret = hardware_encoder_spi_data(&delta);
//	    monitor_prt(&monitor, "%d %f\n", ret, angle, motorQ.state);
	    //monitor_prt(&monitor, "%f\n", angle);
/*
	    if(motorQ.state == MS_IDLE){
	        if(foc_d_q_vec_control(&motorQ, 0.1, 0, ANG2RAD(angle+90))){
	            pwm_sync_duty(motorQ.pwm);
	        }else{
	            monitor_prt(&monitor, "fail\n");
	        }

	    }
	    */
	    hardware_delay_ms(10);

	}

}
