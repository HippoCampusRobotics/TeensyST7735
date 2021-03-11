#pragma once

#include <stdint.h>

#define LED_COLOR_ARMED 0xFF0000
#define LED_COLOR_TIMEOUT 0xFF0058
#define LED_COLOR_READY_TO_FLY 0x00FF00
#define LED_COLOR_NOT_READY 0xFF4000
#define LED_COLOR_ARMING_FAILED 0x0000FF

typedef enum
{
    LED_STATE_READY_TO_FLY,
    LED_STATE_NOT_READY,
    LED_STATE_ARMED,
    LED_STATE_ARMING_FAILED,
    LED_STATE_TIMEOUT,
    LED_UNINIT,
} led_state_t;

void led_init();
void led_set_state(led_state_t state);
void led_update_heartbeat();