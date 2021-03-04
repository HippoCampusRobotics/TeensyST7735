#define USE_FRAME_BUFFER

#define TFT_SCLK 13 // SCLK can also use pin 14
#define TFT_MOSI 11 // MOSI can also use pin 7
#define TFT_CS 10	// CS & DC can use pins 2, 6, 9, 10, 15, 20, 21, 22, 23
#define TFT_DC 9	//  but certain pairs must NOT be used: 2+10, 6+9, 20+23, 21+22
#define TFT_RST 8	// RST can use any pin
#define SD_CS 4		// CS for SD card, can use any pin

#define DT_UPDATE_MS 100
#define ROW_SIZE 14
#define ROW_PADDING 5

// #include <Adafruit_GFX.h>
#include <ST7735_t3.h>
#include <SPI.h>
#include <st7735_t3_font_Arial.h>
#include "standard/mavlink.h"
#include <Fonts/FreeMono9pt7b.h>

typedef struct
{
	float value;
	bool updated;
} value_updated_f_t;

typedef struct
{
	int value;
	bool updated;
} value_updated_d_t;

typedef struct
{
	unsigned value;
	bool updated;
} value_updated_h_t;

typedef struct
{
	value_updated_f_t voltage;
	value_updated_d_t battery_remaining;
	value_updated_f_t roll;
	value_updated_f_t pitch;
	value_updated_f_t yaw;
	value_updated_h_t time_boot_ms;

} screen_data_t;

ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
screen_data_t screen_data;

#define RGB(r, g, b) (b << 11 | g << 6 | r)

#define SETCOLOR(c) disp.setTextColor(c, bg ? ST7735_BLACK : c);

void handle_mavlink_message(mavlink_message_t *msg);
void handle_message_sys_status(mavlink_message_t *msg);
void handle_message_attitude(mavlink_message_t *msg);
void handle_message_system_time(mavlink_message_t *msg);

void update_display();

void setup()
{
	Serial1.begin(921600);
	Serial.begin(921600);

	screen_data.voltage.value = 0.0;
	screen_data.voltage.updated = false;
	screen_data.battery_remaining.value = 0;
	screen_data.battery_remaining.updated = false;
	screen_data.roll.value = 0;
	screen_data.roll.updated = false;
	screen_data.pitch.value = 0;
	screen_data.pitch.updated = false;
	screen_data.yaw.value = 0;
	screen_data.yaw.updated = false;
	screen_data.time_boot_ms.value = 0;
	screen_data.time_boot_ms.updated = false;

	pinMode(SD_CS, INPUT_PULLUP); // keep SD CS high when not using SD card

	// Use this initializer if you're using a 1.8" TFT
	disp.initR(INITR_BLACKTAB);
	disp.useFrameBuffer(true);

	disp.setRotation(1);

	disp.fillRect(0, 0, disp.width(), disp.height(), RGB(0, 0, 0));
	disp.setTextWrap(false);
	disp.setFont(&FreeMono9pt7b);
	disp.setTextColor(RGB(31, 31, 31), RGB(0, 0, 0));
	disp.setCursor(0, 0);
}

void loop()
{
	static float fps = 0.0;
	static uint32_t t_last = 0.0;

	int16_t x, y;
	static mavlink_message_t msg;
	mavlink_status_t status;
	while (Serial1.available() > 0)
	{
		uint8_t byte = Serial1.read();
		if (mavlink_parse_char(0, byte, &msg, &status))
		{
			handle_mavlink_message(&msg);
		}
	}

	uint32_t now = millis();
	if (now - t_last >= DT_UPDATE_MS)
	{
		update_display();
		t_last = now;
	}
}

void update_display()
{
	static int counter = 0;
	counter++;
	char buffer[64];
	int16_t y = ROW_PADDING;
	disp.setCursor(0, 100);
	disp.print("Update: ");
	disp.print(counter);
	disp.setCursor(0, 0);
	disp.setTextColor(ST7735_WHITE, ST7735_BLACK);

	if (screen_data.voltage.updated)
	{
		disp.setCursor(0, y);
		disp.drawRect(0, y, disp.width(), ROW_SIZE, ST7735_BLACK);
		screen_data.voltage.updated = false;
		screen_data.battery_remaining.updated = false;
		disp.print("Batt: ");
		if (screen_data.voltage.value == UINT16_MAX / 1000.0)
		{
			disp.setTextColor(ST77XX_RED, ST77XX_BLACK);
			disp.print("err");
			disp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
		}
		else
			disp.print(screen_data.voltage.value, 1);
		disp.print("|");
		if (screen_data.battery_remaining.value == -1)
		{
			disp.setTextColor(ST77XX_RED, ST77XX_BLACK);
			disp.print("err ");
			disp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
		}
		else
		{
			int val = screen_data.battery_remaining.value;
			disp.setTextColor(ST77XX_RED, ST77XX_BLACK);
			if (val > 30)
				disp.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
			if (val > 60)
				disp.setTextColor(ST77XX_GREEN, ST77XX_BLACK);

			sprintf(buffer, "%3d", screen_data.battery_remaining.value);
			disp.print(buffer);
			disp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
		}
	}

	y += ROW_PADDING + ROW_SIZE;

	if (screen_data.roll.updated)
	{
		screen_data.roll.updated = false;
		screen_data.pitch.updated = false;
		screen_data.yaw.updated = false;
		disp.setCursor(0, y);
		sprintf(buffer, "Yaw: %4d(deg)", (int)(screen_data.yaw.value * 180.0 / PI));
		disp.drawRect(0, y, disp.width(), ROW_SIZE, ST7735_BLACK);
		disp.print(buffer);
	}

	y += ROW_PADDING + ROW_SIZE;

	if (screen_data.time_boot_ms.updated)
	{
		screen_data.time_boot_ms.updated = false;
		disp.setCursor(0, y);
		sprintf(buffer, "Uptime: %5ds", (int)(screen_data.time_boot_ms.value / 1000.0));
		disp.print(buffer);
	}
	disp.updateScreenAsync();
}

void handle_mavlink_message(mavlink_message_t *msg)
{
	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_SYS_STATUS:
		handle_message_sys_status(msg);
		break;
	case MAVLINK_MSG_ID_ATTITUDE:
		handle_message_attitude(msg);
		break;
	case MAVLINK_MSG_ID_SYSTEM_TIME:
		handle_message_system_time(msg);
		break;
	default:
		break;
	}
}

void handle_message_sys_status(mavlink_message_t *msg)
{
	mavlink_sys_status_t sys_status;
	mavlink_msg_sys_status_decode(msg, &sys_status);
	screen_data.voltage.value = sys_status.voltage_battery / 1000.0;
	screen_data.voltage.updated = true;
	screen_data.battery_remaining.value = sys_status.battery_remaining;
	screen_data.battery_remaining.updated = true;
}

void handle_message_attitude(mavlink_message_t *msg)
{
	mavlink_attitude_t attitude;
	mavlink_msg_attitude_decode(msg, &attitude);
	screen_data.roll.value = attitude.roll;
	screen_data.roll.updated = true;
	screen_data.pitch.value = attitude.pitch;
	screen_data.pitch.updated = true;
	screen_data.yaw.value = attitude.yaw;
	screen_data.yaw.updated = true;
}

void handle_message_system_time(mavlink_message_t *msg)
{
	mavlink_system_time_t system_time;
	mavlink_msg_system_time_decode(msg, &system_time);
	screen_data.time_boot_ms.value = system_time.time_boot_ms;
	screen_data.time_boot_ms.updated = true;
}