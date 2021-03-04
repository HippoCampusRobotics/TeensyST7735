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
	value_updated_h_t base_mode;
	value_updated_h_t custom_mode;
	value_updated_h_t state;

} screen_data_t;

union custom_mode
{
	enum MAIN_MODE : uint8_t
	{
		MAIN_MODE_MANUAL = 1,
		MAIN_MODE_ALTCTL,
		MAIN_MODE_POSCTL,
		MAIN_MODE_AUTO,
		MAIN_MODE_ACRO,
		MAIN_MODE_OFFBOARD,
		MAIN_MODE_STABILIZED,
		MAIN_MODE_RATTITUDE
	};

	enum SUB_MODE_AUTO : uint8_t
	{
		SUB_MODE_AUTO_READY = 1,
		SUB_MODE_AUTO_TAKEOFF,
		SUB_MODE_AUTO_LOITER,
		SUB_MODE_AUTO_MISSION,
		SUB_MODE_AUTO_RTL,
		SUB_MODE_AUTO_LAND,
		SUB_MODE_AUTO_RTGS,
		SUB_MODE_AUTO_FOLLOW_TARGET,
		SUB_MODE_AUTO_PRECLAND
	};

	struct
	{
		uint16_t reserved;
		uint8_t main_mode;
		uint8_t sub_mode;
	};
	uint32_t data;
	float data_float;

	custom_mode() : data(0)
	{
	}

	explicit custom_mode(uint32_t val) : data(val)
	{
	}

	constexpr custom_mode(uint8_t mm, uint8_t sm) : reserved(0),
													main_mode(mm),
													sub_mode(sm)
	{
	}
};

/**
 * @brief helper function to define any mode as uint32_t constant
 *
 * @param mm main mode
 * @param sm sub mode (currently used only in auto mode)
 * @return uint32_t representation
 */
constexpr uint32_t define_mode(enum custom_mode::MAIN_MODE mm, uint8_t sm = 0)
{
	return custom_mode(mm, sm).data;
}

/**
 * @brief helper function to define auto mode as uint32_t constant
 *
 * Same as @a define_mode(custom_mode::MAIN_MODE_AUTO, sm)
 *
 * @param sm auto sub mode
 * @return uint32_t representation
 */
constexpr uint32_t define_mode_auto(enum custom_mode::SUB_MODE_AUTO sm)
{
	return define_mode(custom_mode::MAIN_MODE_AUTO, sm);
}

ST7735_t3 disp = ST7735_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
screen_data_t screen_data{};

#define RGB(r, g, b) (b << 11 | g << 6 | r)

#define SETCOLOR(c) disp.setTextColor(c, bg ? ST7735_BLACK : c);

void handle_mavlink_message(mavlink_message_t *msg);
void handle_message_sys_status(mavlink_message_t *msg);
void handle_message_attitude(mavlink_message_t *msg);
void handle_message_system_time(mavlink_message_t *msg);
void handle_message_heartbeat(mavlink_message_t *msg);

void init_serial();
void init_display();
void update_display();
void mavlink_send_heartbeat();

void setup()
{
	init_serial();
	init_display();
}

void loop()
{
	static float fps = 0.0;
	static uint32_t t_last = 0.0;

	int16_t x, y;
	static mavlink_message_t msg;
	mavlink_status_t status;
	while (MAVLINK_SERIAL.available() > 0)
	{
		uint8_t byte = MAVLINK_SERIAL.read();
		if (mavlink_parse_char(0, byte, &msg, &status))
		{
			handle_mavlink_message(&msg);
		}
	}

	uint32_t now = millis();
	if (now - t_last >= DT_UPDATE_MS)
	{
		mavlink_send_heartbeat();
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

	y += ROW_PADDING + ROW_SIZE;

	if (screen_data.state.updated)
	{
		disp.fillRect(0, y, disp.width(), ROW_SIZE, ST7735_BLACK);
		screen_data.state.updated = false;
		disp.setCursor(0, y);
		disp.print("State: ");
		switch (screen_data.state.value)
		{
		case MAV_STATE_UNINIT:
			disp.print("UNINIT");
			break;
		case MAV_STATE_BOOT:
			disp.print("BOOT");
			break;
		case MAV_STATE_CALIBRATING:
			disp.print("CALIB");
			break;
		case MAV_STATE_STANDBY:
			disp.print("STANDBY");
			break;
		case MAV_STATE_ACTIVE:
			disp.print("ACTIVE");
			break;
		case MAV_STATE_CRITICAL:
			disp.print("CRIT");
			break;
		case MAV_STATE_EMERGENCY:
			disp.print("EMERG");
			break;
		case MAV_STATE_POWEROFF:
			disp.print("OFF");
			break;
		case MAV_STATE_FLIGHT_TERMINATION:
			disp.print("TERMIN");
			break;
		default:
			disp.print("?");
			break;
		}
	}

	y += ROW_PADDING + ROW_SIZE;

	if (screen_data.custom_mode.updated)
	{
		screen_data.custom_mode.updated = false;
		disp.fillRect(0, y, disp.width(), ROW_SIZE, ST7735_BLACK);
		disp.setCursor(0, y);
		custom_mode mode(screen_data.custom_mode.value);
		if (mode.main_mode == custom_mode::MAIN_MODE_AUTO)
		{
			switch (mode.sub_mode)
			{
			case custom_mode::SUB_MODE_AUTO_LAND:
				disp.print("LAND");
				break;
			default:
				disp.print("SUB ???");
				break;
			}
		}
		else
		{
			switch (mode.main_mode)
			{
			case custom_mode::MAIN_MODE_OFFBOARD:
				disp.print("OFFBOARD");
				break;
			case custom_mode::MAIN_MODE_MANUAL:
				disp.print("MANUAL");
				break;
			default:
				disp.print("MAIN???");
				break;
			}
		}
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
	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
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

void handle_message_heartbeat(mavlink_message_t *msg)
{
	mavlink_heartbeat_t heartbeat;
	mavlink_msg_heartbeat_decode(msg, &heartbeat);
	screen_data.base_mode.value = heartbeat.base_mode;
	screen_data.base_mode.updated = true;
	screen_data.custom_mode.value = heartbeat.custom_mode;
	screen_data.custom_mode.updated = true;
	screen_data.state.value = heartbeat.system_status;
	screen_data.state.updated = true;
}

void init_serial()
{
	MAVLINK_SERIAL.setTX(MAVLINK_TX);
	MAVLINK_SERIAL.setRX(MAVLINK_RX);
	// MAVLINK_SERIAL.attachRts(MAVLINK_RTS);
	// MAVLINK_SERIAL.attachCts(MAVLINK_CTS);
	MAVLINK_SERIAL.begin(MAVLINK_BAUD);
}

void init_display()
{
	pinMode(SD_CS, INPUT_PULLUP); // keep SD CS high when not using SD card

	// Use this initializer if you're using a 1.8" TFT
	disp.initR(INITR_BLACKTAB);
	disp.useFrameBuffer(true);

	disp.setRotation(1);

	disp.fillRect(0, 0, disp.width(), disp.height(), RGB(0, 0, 0));
	disp.setTextWrap(false);
	disp.setFont(&FreeMono9pt7b);
	// disp.setFont(Arial_10);
	disp.setTextColor(RGB(31, 31, 31), RGB(0, 0, 0));
	disp.setCursor(0, 0);
}

void mavlink_send_heartbeat()
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(1, 25, &msg, MAV_TYPE::MAV_TYPE_GENERIC, MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, MAV_MODE::MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE::MAV_STATE_STANDBY);

	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
	MAVLINK_SERIAL.write(buffer, len);
}
