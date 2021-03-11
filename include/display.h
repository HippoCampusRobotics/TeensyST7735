#ifndef __display_H_
#define __display_H_

#include <ST7735_t3.h>
#include "custom_mode.h"
#include <st7735_t3_font_Arial.h>

#define DISPLAY_COLOR_ARMED ST7735_RED
#define DISPLAY_COLOR_DISARMED ST7735_GREEN
#define DISPLAY_COLOR_NORMAL ST7735_WHITE
#define DISPLAY_COLOR_ERR ST7735_RED
#define DISPLAY_BATTERY_HIGH ST7735_GREEN
#define DISPLAY_BATTERY_MED ST7735_YELLOW
#define DISPLAY_BATTERY_LOW ST7735_RED
#define DISPLAY_COLOR_BACKGROUND ST7735_BLACK

#define ROW_HEIGHT 17
#define ROW_PADDING 3
#define STANDARD_FONT Arial_12
#define POSITION_FONT Arial_11

typedef struct
{
    float x;
    float y;
    float z;
} position_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} orientation_t;

typedef struct
{
    bool updated;
    uint32_t stamp;
} update_status_t;

class DisplayManager
{
    ST7735_t3 *_display;
    position_t _position {};
    update_status_t _position_update {};
    orientation_t _orientation {};
    update_status_t _orientation_update {};
    int _battery_remaining;
    update_status_t _battery_remaining_update {};
    float _voltage;
    update_status_t _voltage_update {};
    uint32_t _time_boot_ms;
    update_status_t _time_boot_ms_update {};
    custom_mode _mode;
    update_status_t _mode_update {};
    uint32_t _state;
    update_status_t _state_update {};
    bool _prearm_check_passed;
    update_status_t _prearm_check_passed_update {};

    inline void delete_line()
    {
        int16_t y;
        y = _display->getCursorY();
        _display->fillRect(0, y-2, _display->width(), ROW_HEIGHT+2, DISPLAY_COLOR_BACKGROUND);
    }

    inline void check_stamp(uint32_t stamp, update_status_t &update_status)
    {
        if (stamp >= update_status.stamp)
            update_status.updated = true;
        update_status.stamp = stamp;
    }

public:
    DisplayManager()
    {
        _display = NULL;
    }
    DisplayManager(ST7735_t3 *display)
    {
        _display = display;
    }
    void update_position(float x, float y, float z, uint32_t stamp);
    void update_orientation(int roll, int pitch, int yaw, uint32_t stamp);
    void update_battery_remaining(int remaining, uint32_t stamp);
    void update_voltage(float voltage, uint32_t stamp);
    void update_time_boot_ms(uint32_t time_boot_ms, uint32_t stamp);
    void update_prearm_check(bool prearm_check_passed, uint32_t stamp);
    void update_mode(custom_mode mode, uint32_t stamp);
    void update_state(uint32_t state, uint32_t stamp);

    position_t get_position(bool do_not_consume = false);
    orientation_t get_orientation(bool do_not_consume = false);
    int get_battery_remaining(bool do_not_consume = false);
    float get_voltage(bool do_not_consume = false);
    uint32_t get_time_boot_ms(bool do_not_consume = false);
    custom_mode get_mode(bool do_not_consume = false);
    uint32_t get_state(bool do_not_consume = false);
    bool get_prearm_check(bool do_not_consume = false);

    inline bool position_is_updated() { return _position_update.updated; };
    inline bool orientation_is_updated() { return _orientation_update.updated; };
    inline bool battery_remaining_is_updated() { return _battery_remaining_update.updated; };
    inline bool voltage_is_updated() { return _voltage_update.updated; };
    inline bool time_boot_ms_is_updated() { return _time_boot_ms_update.updated; };
    inline bool prearm_check_is_updated() { return _prearm_check_passed_update.updated; };
    inline bool mode_is_updated() { return _mode_update.updated; };
    inline bool state_is_updated() { return _state_update.updated; };

    void print_position();
    void print_orientation();
    void print_battery_status();
    void print_uptime();
    void print_mode();
    void print_state();
};

#endif