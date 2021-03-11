#include "led.h"
#include <TeensyTimerTool.h>
#include <WS2812Serial.h>
static TeensyTimerTool::PeriodicTimer heartbeat_timeout_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer led_switch_on_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::OneShotTimer led_switch_off_timer(TeensyTimerTool::TCK);

static const int num_led = 3;
static const int led_pin = 1;
static byte led_draw_buffer[num_led * 3];
static DMAMEM byte led_display_buffer[num_led * 12];
static WS2812Serial leds(num_led, led_display_buffer, led_draw_buffer, led_pin, WS2812_GRB);
static uint32_t led_color;
static led_state_t _state;
static bool heartbeat_timed_out = false;

void set_all_leds_color(int color);
void heartbeat_timeout_cb();
void led_switch_on_cb();
void led_switch_off_cb();
void led_set_color(uint32_t color);

const uint32_t blink_period_armed = 500'000;
const uint32_t switch_off_delay_armed = 100'000;
const uint32_t blink_period_timeout = 1'000'000;
const uint32_t switch_off_delay_timeout = 500'000;

void led_update_heartbeat()
{
    heartbeat_timed_out = false;
}

inline void led_set_new_period(uint32_t period, bool start = true)
{
    led_switch_on_timer.begin(led_switch_on_cb, period, start);
}

inline void led_switch_immediately(uint32_t color, uint32_t period = 0)
{

    led_switch_off_timer.stop();
    if (period)
    {
        led_set_new_period(period, false);
        set_all_leds_color(color);
        led_switch_on_timer.start();
    }
    else
    {

        led_switch_on_timer.stop();
        led_switch_on_timer.start();
    }
    led_switch_on_cb();
}

void led_init()
{
    leds.begin();
    _state = LED_UNINIT;
    heartbeat_timeout_timer.begin(heartbeat_timeout_cb, 1'500'000);
    led_switch_on_timer.begin(led_switch_on_cb, blink_period_timeout);
    led_switch_off_timer.begin(led_switch_off_cb);
    led_set_state(LED_STATE_TIMEOUT);
}

void led_set_state(led_state_t state)
{
    if (state == LED_STATE_ARMING_FAILED)
    {
        led_switch_immediately(LED_COLOR_ARMING_FAILED);
    }
    else
    {
        if (state == _state)
            return;
        _state = state;
        if (state == LED_STATE_ARMED)
        {
            led_color = LED_COLOR_ARMED;
            led_switch_immediately(led_color, blink_period_armed);
        }
        else if (state == LED_STATE_NOT_READY)
        {
            led_color = LED_COLOR_NOT_READY;
            led_switch_immediately(led_color);
        }
        else if (state == LED_STATE_READY_TO_FLY)
        {
            led_color = LED_COLOR_READY_TO_FLY;
            led_switch_immediately(led_color);
        }
        else if (state == LED_STATE_TIMEOUT)
        {
            led_color = LED_COLOR_TIMEOUT;
            led_switch_immediately(led_color, blink_period_timeout);
        }
    }
}

void set_all_leds_color(int color)
{
    for (int i = 0; i < leds.numPixels(); i++)
    {
        leds.setPixel(i, color);
    }
    leds.show();
}

void heartbeat_timeout_cb()
{
    if (heartbeat_timed_out)
    {
        led_set_state(LED_STATE_TIMEOUT);
    }
    else
    {
        heartbeat_timed_out = true;
    }
}

void led_switch_on_cb()
{
    Serial.println("Switch on LED");
    Serial.print(led_color, 16);
    set_all_leds_color(led_color);
    switch (_state)
    {
    case LED_STATE_ARMED:
        led_switch_off_timer.trigger(switch_off_delay_armed);
        break;
    case LED_STATE_TIMEOUT:
        led_switch_off_timer.trigger(switch_off_delay_timeout);
        break;
    default:
        led_switch_off_timer.stop();
        break;
    }
}

void led_switch_off_cb()
{
    set_all_leds_color(0);
}
