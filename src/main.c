// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bt_hid.h"

// These magic values are just taken from M0o+, not calibrated for
// the Tiny chassis.
#define PWM_WRAP 6048
#define PWM_MID 936  //  decodes 1500ms
#define PWM_1000MS 620
#define THROTTLE_ABS_MAX 512  //  255
#define STEERING_ABS_MAX 186  // 127
#define PWM_MAX (PWM_MID + PWM_1000MS)
#define PWM_MIN (PWM_MID - PWM_1000MS)

static inline int8_t clamp8(int16_t value) {
        if (value > 127) {
                return 127;
        } else if (value < -128) {
                return -128;
        }

        return value;
}

struct slice {
	unsigned int slice_num;
	unsigned int pwm_min;
};

struct chassis {
	struct slice slice_l;
	struct slice slice_r;

	int8_t l;
	int8_t r;
};

void init_slice(struct slice *slice, unsigned int slice_num, unsigned int pwm_min, uint8_t pin_a)
{
        // Count once for every 100 cycles the PWM B input is high
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
        pwm_config_set_clkdiv(&cfg, 200);
        pwm_init(slice_num, &cfg, false);


	slice->slice_num = slice_num;
	slice->pwm_min = pwm_min;
	gpio_set_function(pin_a, GPIO_FUNC_PWM);
	gpio_set_function(pin_a + 1, GPIO_FUNC_PWM);
	pwm_set_wrap(slice->slice_num, PWM_WRAP);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_A, PWM_MID);
	pwm_set_chan_level(slice->slice_num, PWM_CHAN_B, PWM_MID);
	pwm_set_enabled(slice->slice_num, true);
}

void chassis_init(struct chassis *chassis, uint8_t pin_la, uint8_t pin_ra)
{

	init_slice(&chassis->slice_l, pwm_gpio_to_slice_num(pin_la), PWM_WRAP, pin_la);
	init_slice(&chassis->slice_r, pwm_gpio_to_slice_num(pin_ra), PWM_WRAP, pin_ra);
}

static inline uint16_t abs16(int16_t v) {
	return v < 0 ? -v : v;
}

void slice_set(struct slice *slice, int16_t value)
{
	pwm_set_both_levels(slice->slice_num, PWM_MID+value, PWM_MID+value);
}

void chassis_set_raw(struct chassis *chassis, int16_t throttle, int16_t steering)
{
	slice_set(&chassis->slice_l, throttle);
	slice_set(&chassis->slice_r, steering);

	chassis->l = throttle;
	chassis->r = steering;
}

void chassis_set(struct chassis *chassis, int16_t throttle, int16_t steering)
{
	chassis_set_raw(chassis, throttle, steering);
}

int16_t smooth_center_steering(int16_t input, int16_t promil)
{
    if( abs16(input) < promil*STEERING_ABS_MAX/1000 )
    {
      return 0;
    }
    else
    {
      return input;
    }
}

void main(void) {
	stdio_init_all();

	sleep_ms(1000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);
	// Wait for init (should do a handshake with the fifo here?)
	sleep_ms(1000);

	struct chassis chassis = { 0 };
	chassis_init(&chassis, 6, 8);

	struct bt_hid_state state;
	for ( ;; ) {
		sleep_ms(20);
		bt_hid_get_latest(&state);
		//  float speed_scale = 1.0;
                static bool armed = false;
                if ( state.l2 == 255 && state.r2 == 255 )
                {
                  armed = true;
                }
		int16_t throttle = -PWM_1000MS*(state.l2 - state.r2)/THROTTLE_ABS_MAX;
		int16_t steering = -PWM_1000MS*clamp8(state.rx - 128)/STEERING_ABS_MAX;
                steering = smooth_center_steering(steering, 40); // promils
                if (!armed){
                  throttle = 0;
                  steering = 0;
                }
		chassis_set(&chassis, throttle, steering);

		printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d, |%d:%d|\n",
				state.buttons, state.lx, state.ly, state.rx, state.ry,
				state.l2, state.r2, state.hat, throttle, steering);

	}
}
