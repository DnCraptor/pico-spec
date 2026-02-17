#include <stdio.h>
#include <pico.h>
#include <pico/platform.h>
#include <hardware/clocks.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <hardware/structs/systick.h>
#include "PinSerialData_595.h"

static volatile bool ts_595_enabled = false;

static void PWM_init_pin(uint pinN){
    gpio_set_function(pinN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pinN);
	//printf("595 slice_num:%d\n",slice_num);
	pwm_config c_pwm=pwm_get_default_config();
	pwm_config_set_clkdiv(&c_pwm,clock_get_hz(clk_sys)/(4.0*1750000));
	pwm_config_set_wrap(&c_pwm,3);//MAX PWM value
	pwm_init(slice_num,&c_pwm,true);
}

static void PWM_deinit_pin(uint pinN){
	if ( gpio_get_function(pinN) == GPIO_FUNC_PWM ) {
    	uint slice_num = pwm_gpio_to_slice_num(pinN);
		//printf("595 deslice_num:%d\n",slice_num);
		pwm_set_enabled(slice_num, false);
		pwm_config c_pwm = pwm_get_default_config();
		pwm_init(slice_num, &c_pwm, false);
		gpio_deinit(pinN);
	}
}

void Init_PWM_175(uint8_t tspin_mode){
	if (!ts_595_enabled) {
		//printf("Init 595\n");
		ts_595_enabled = true;
		switch (tspin_mode) {
			case TSPIN_MODE_GP21:
				PWM_init_pin(CLK_AY_PIN1);
				pwm_set_gpio_level(CLK_AY_PIN1, 2);
				break;
			case TSPIN_MODE_GP29:
				PWM_init_pin(CLK_AY_PIN2);
				pwm_set_gpio_level(CLK_AY_PIN2, 2);
				break;
			case TSPIN_MODE_BOTH:
				PWM_init_pin(CLK_AY_PIN1);
				pwm_set_gpio_level(CLK_AY_PIN1, 2);
				PWM_init_pin(CLK_AY_PIN2);
				pwm_set_gpio_level(CLK_AY_PIN2, 2);
				break;
		}
		gpio_init(CLK_595_PIN);
		gpio_set_dir(CLK_595_PIN,GPIO_OUT);
		gpio_init(DATA_595_PIN);
		gpio_set_dir(DATA_595_PIN,GPIO_OUT);
		gpio_init(LATCH_595_PIN);
		gpio_set_dir(LATCH_595_PIN,GPIO_OUT);
	}
}

void Deinit_PWM_175() {
	if (ts_595_enabled) {
		//printf("DeInit 595\n");
		PWM_deinit_pin(CLK_AY_PIN1);
		PWM_deinit_pin(CLK_AY_PIN2);
		gpio_deinit(CLK_595_PIN);
		gpio_deinit(DATA_595_PIN);
		gpio_deinit(LATCH_595_PIN);
		ts_595_enabled = false;
	}
}

#define _30MHZ 30000000

inline static void wait_to_adjust(uint32_t wait_nops) {
	for (int i = 0; i < wait_nops; ++i) __asm volatile("nop");
}

void __not_in_flash_func(send_to_595)(uint16_t data) {
	static uint32_t wait_nops = 0;
	if (wait_nops == 0) {
     	wait_nops = clock_get_hz(clk_sys) / (_30MHZ * 5);
	}
	gpio_put(CLK_595_PIN, 0);
	wait_to_adjust(wait_nops);
	for(int i = 0; i < 16; ++i) { 
		gpio_put(DATA_595_PIN, (0x8000 & data));
		data <<= 1;
	    gpio_put(CLK_595_PIN, 1);
		wait_to_adjust(wait_nops);
	    gpio_put(CLK_595_PIN, 0);
		wait_to_adjust(wait_nops);
	}
    gpio_put(LATCH_595_PIN, 1);
	wait_to_adjust(wait_nops);
	gpio_put(LATCH_595_PIN, 0);
}

uint16_t control_bits = 0;

void __not_in_flash_func(AY_to595Beep)(bool Beep){ 
	if (Beep) send_to_595( HIGH(Beeper));
	else send_to_595( LOW(Beeper));
};
