#define USE_FRAME_BUFFER

#define TFT_SCLK 13 // SCLK can also use pin 14
#define TFT_MOSI 11 // MOSI can also use pin 7
#define TFT_CS 10	// CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC 9	//  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST 8	// RST can use any pin
#define SD_CS 4		// CS for SD card, can use any pin

#define MAVLINK_SERIAL Serial3
#define MAVLINK_RX 15
#define MAVLINK_TX 14
#define MAVLINK_CTS 19
#define MAVLINK_RTS 18
#define MAVLINK_BAUD 921600

#define DT_UPDATE_MS 100

#define LED_ARMED 0xFF0000
#define LED_READY_TO_FLY 0x00FF00
#define LED_NOT_READY 0xFF0058;
#define LED_HEARTBEAT_TIMEOUT 0xFF3000
#define LED_ARMING_FAILED 0x0000FF



// #include <Adafruit_GFX.h>
#include <ST7735_t3.h>
#include <st7735_t3_font_Arial.h>
#include "standard/mavlink.h"
#include "px4_interface.h"
#include "display.h"
#include <WS2812Serial.h>
#include <TeensyTimerTool.h>

const int num_led = 3;
const int led_pin = 1;

byte led_draw_buffer[num_led * 3];
DMAMEM byte led_display_buffer[num_led * 12];

WS2812Serial leds(num_led, led_display_buffer, led_draw_buffer, led_pin, WS2812_GRB);

uint32_t led_color;


ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
DisplayManager display_manager(&disp);

TeensyTimerTool::Timer heartbeat_timeout_timer;
TeensyTimerTool::Timer led_switch_on_timer;
TeensyTimerTool::OneShotTimer led_switch_off_timer;
bool heartbeat_timed_out = false;

void init_serial();
void init_display();
void update_display();
void set_all_leds_color(int color);
void heartbeat_timeout_cb();
void led_switch_on_cb();
void led_switch_off_cb();

void setup()
{
	init_serial();
	init_display();
	leds.begin();
	set_all_leds_color(LED_HEARTBEAT_TIMEOUT);
	heartbeat_timeout_timer.beginPeriodic(heartbeat_timeout_cb, 3'000'000);
	led_switch_on_timer.beginPeriodic(led_switch_on_cb, 500'000);
	led_switch_off_timer.begin(led_switch_off_cb);
}

void loop()
{
	static uint32_t t_last = 0.0;
	static mavlink_message_t msg;
	mavlink_status_t status;
	while (MAVLINK_SERIAL.available() > 0)
	{
		uint8_t byte = MAVLINK_SERIAL.read();
		if (mavlink_parse_char(0, byte, &msg, &status))
		{
			px4_interface::handle_mavlink_message(&msg);
		}
	}

	uint32_t now = millis();
	if (now - t_last >= DT_UPDATE_MS)
	{
		px4_interface::send_heartbeat();
		update_display();
		t_last = now;
	}
}


void update_display()
{
	disp.setCursor(3, 0);
	disp.setTextColor(DISPLAY_COLOR_NORMAL);

	int16_t y = ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_state();

	y += ROW_HEIGHT;
	disp.drawFastHLine(0, y, disp.width(), DISPLAY_COLOR_NORMAL);
	y += ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_mode();

	y += ROW_HEIGHT;
	disp.drawFastHLine(0, y, disp.width(), DISPLAY_COLOR_NORMAL);
	y += ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_battery_status();

	y += ROW_HEIGHT;
	disp.drawFastHLine(0, y, disp.width(), DISPLAY_COLOR_NORMAL);
	y += ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_uptime();

	y += ROW_PADDING + ROW_HEIGHT;
	disp.fillRect(0, y-ROW_PADDING, disp.width(), ROW_PADDING, DISPLAY_COLOR_NORMAL);
	y += ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_position();
	
	y += ROW_HEIGHT;
	disp.drawFastHLine(0, y, disp.width(), DISPLAY_COLOR_NORMAL);
	y += ROW_PADDING;
	disp.setCursor(3, y);
	display_manager.print_orientation();
	

	disp.updateScreenAsync();
}

void init_serial()
{
	MAVLINK_SERIAL.setTX(MAVLINK_TX);
	MAVLINK_SERIAL.setRX(MAVLINK_RX);
	// MAVLINK_SERIAL.attachRts(MAVLINK_RTS);
	// MAVLINK_SERIAL.attachCts(MAVLINK_CTS);
	MAVLINK_SERIAL.begin(MAVLINK_BAUD);
	px4_interface::set_display_manager(&display_manager);
	px4_interface::set_mavlink_port(&MAVLINK_SERIAL);
	Serial.begin(115200);

}

void init_display()
{
	pinMode(SD_CS, INPUT_PULLUP); // keep SD CS high when not using SD card

	// Use this initializer if you're using a 1.8" TFT
	disp.initR(INITR_BLACKTAB);
	disp.useFrameBuffer(true);

	disp.setRotation(1);

	disp.fillScreen(DISPLAY_COLOR_BACKGROUND);
	disp.setTextWrap(false);
	// disp.setFont(&FreeMono9pt7b);
	disp.setFont(STANDARD_FONT);
	disp.setTextColor(DISPLAY_COLOR_NORMAL);
	disp.setCursor(0, 0);
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
		led_color = LED_HEARTBEAT_TIMEOUT;
	}
	else
	{
		heartbeat_timed_out = true;
	}
}

void led_switch_on_cb()
{
	set_all_leds_color(led_color);
	if (led_color == LED_ARMED)
		led_switch_off_timer.trigger(100'000);
}

void led_switch_off_cb()
{
	set_all_leds_color(0);
}