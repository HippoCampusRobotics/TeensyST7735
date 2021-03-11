#include "display.h"
#include "standard/mavlink.h"

void DisplayManager::update_position(float x, float y, float z, uint32_t stamp)
{
    _position.x = x;
    _position.y = y;
    _position.z = z;
    check_stamp(stamp, _position_update);
}
void DisplayManager::update_orientation(int roll, int pitch, int yaw, uint32_t stamp)
{
    _orientation.roll = roll;
    _orientation.pitch = pitch;
    _orientation.yaw = yaw;
    check_stamp(stamp, _orientation_update);
}

void DisplayManager::update_battery_remaining(int remaining, uint32_t stamp)
{
    _battery_remaining = remaining;
    check_stamp(stamp, _battery_remaining_update);
}

void DisplayManager::update_voltage(float voltage, uint32_t stamp)
{
    _voltage = voltage;
    check_stamp(stamp, _voltage_update);
}

void DisplayManager::update_time_boot_ms(uint32_t time_boot_ms, uint32_t stamp)
{
    _time_boot_ms = time_boot_ms;
    check_stamp(stamp, _time_boot_ms_update);
}

void DisplayManager::update_prearm_check(bool prearm_check_passed, uint32_t stamp)
{
    _prearm_check_passed = prearm_check_passed;
    check_stamp(stamp, _prearm_check_passed_update);
}

void DisplayManager::update_mode(custom_mode mode, uint32_t stamp)
{
    _mode = mode;
    check_stamp(stamp, _mode_update);
}

void DisplayManager::update_state(uint32_t state, uint32_t stamp)
{
    _state = state;
    check_stamp(stamp, _state_update);
}

position_t DisplayManager::get_position(bool do_not_consume)
{
    if (!do_not_consume)
        _position_update.updated = false;
    return _position;
}

orientation_t DisplayManager::get_orientation(bool do_not_consume)
{
    if (!do_not_consume)
        _orientation_update.updated = false;
    return _orientation;
}

int DisplayManager::get_battery_remaining(bool do_not_consume)
{
    if (!do_not_consume)
        _battery_remaining_update.updated = false;
    return _battery_remaining;
}

float DisplayManager::get_voltage(bool do_not_consume)
{
    if (!do_not_consume)
        _voltage_update.updated = false;
    return _voltage;
}
uint32_t DisplayManager::get_time_boot_ms(bool do_not_consume)
{
    if (!do_not_consume)
        _time_boot_ms_update.updated = false;
    return _time_boot_ms;
}
custom_mode DisplayManager::get_mode(bool do_not_consume)
{
    if (!do_not_consume)
        _mode_update.updated = false;
    return _mode;
}
uint32_t DisplayManager::get_state(bool do_not_consume)
{
    if (!do_not_consume)
        _state_update.updated = false;
    return _state;
}
bool DisplayManager::get_prearm_check(bool do_not_consume)
{
    if (!do_not_consume)
        _prearm_check_passed_update.updated = false;
    return _prearm_check_passed;
}

void DisplayManager::print_position()
{
    if (!(_display && position_is_updated()))
        return;
    delete_line();
    position_t pos = get_position();
    _display->print("Pos: ");
    _display->setFont(POSITION_FONT);
    _display->print(pos.x, 2);
    _display->print(" | ");
    _display->print(pos.y, 2);
    _display->print(" | ");
    _display->print(pos.z, 2);
    _display->setFont(STANDARD_FONT);
}
void DisplayManager::print_orientation()
{
    if (!(_display))
        return;
    delete_line();
    char buffer[32];
    orientation_t rpy = get_orientation();
    _display->print("RPY: ");
    snprintf(buffer, 32, "%4d | %4d | %4d", (int)rpy.roll, (int)rpy.pitch, (int)rpy.yaw);
    _display->setFont(POSITION_FONT);
    _display->print(buffer);
    _display->setFont(STANDARD_FONT);
}

void DisplayManager::print_battery_status()
{
    if (!(_display && voltage_is_updated()))
        return;
    delete_line();
    float voltage = get_voltage();
    int remaining = get_battery_remaining();
    _display->print("Batt: ");
    if (voltage == UINT16_MAX / 1000.0)
    {
        _display->setTextColor(DISPLAY_COLOR_ERR);
        _display->print("err");
        _display->setTextColor(DISPLAY_COLOR_NORMAL);
    }
    else
    {
        _display->print(voltage, 2);
        _display->print(" V");
    }
    _display->print(" | ");
    if (remaining == -1)
    {
        _display->setTextColor(DISPLAY_COLOR_ERR);
        _display->print("err");
        _display->setTextColor(DISPLAY_COLOR_NORMAL);
    }
    else
    {
        _display->setTextColor(DISPLAY_BATTERY_LOW);
        if (remaining > 30)
            _display->setTextColor(DISPLAY_BATTERY_MED);
        if (remaining > 60)
            _display->setTextColor(DISPLAY_BATTERY_HIGH);
        _display->print(remaining);
        _display->print("%");
    }
    _display->setTextColor(DISPLAY_COLOR_NORMAL);
}
void DisplayManager::print_uptime()
{
    if (!(_display && time_boot_ms_is_updated()))
        return;
    delete_line();
    char buffer[32];
    snprintf(buffer, 32, "Uptime: %5d s", (int)(get_time_boot_ms() / 1000));
    _display->print(buffer);
}
void DisplayManager::print_mode()
{
    if (!(_display && mode_is_updated()))
        return;
    delete_line();
    custom_mode mode = get_mode();
    _display->print("Mode: ");
    if (mode.main_mode == custom_mode::MAIN_MODE_AUTO)
    {
        switch (mode.sub_mode)
        {
        case custom_mode::SUB_MODE_AUTO_LAND:
            _display->print("LAND");
            break;
        case custom_mode::SUB_MODE_AUTO_READY:
            _display->print("READY");
            break;
        case custom_mode::SUB_MODE_AUTO_TAKEOFF:
            _display->print("TAKEOFF");
            break;
        case custom_mode::SUB_MODE_AUTO_LOITER:
            _display->print("LOITER");
            break;
        case custom_mode::SUB_MODE_AUTO_MISSION:
            _display->print("MISSION");
            break;
        case custom_mode::SUB_MODE_AUTO_RTL:
            _display->print("RTL");
            break;
        case custom_mode::SUB_MODE_AUTO_RTGS:
            _display->print("RTGS");
            break;
        case custom_mode::SUB_MODE_AUTO_FOLLOW_TARGET:
            _display->print("FOLLOW");
            break;
        case custom_mode::SUB_MODE_AUTO_PRECLAND:
            _display->print("PRECLAND");
            break;
        default:
            _display->print("SUB ???");
            break;
        }
    }
    else
    {
        switch (mode.main_mode)
        {
        case custom_mode::MAIN_MODE_OFFBOARD:
            _display->print("OFFBOARD");
            break;
        case custom_mode::MAIN_MODE_MANUAL:
            _display->print("MANUAL");
            break;
        case custom_mode::MAIN_MODE_ALTCTL:
            _display->print("ALTCTL");
            break;
        case custom_mode::MAIN_MODE_POSCTL:
            _display->print("POSCTL");
            break;
        case custom_mode::MAIN_MODE_ACRO:
            _display->print("ACRO");
            break;
        case custom_mode::MAIN_MODE_STABILIZED:
            _display->print("STABILIZED");
            break;
        case custom_mode::MAIN_MODE_RATTITUDE:
            _display->print("RATT");
            break;
        default:
            _display->print("MAIN???");
            break;
        }
    }
}
void DisplayManager::print_state()
{
    if (!(_display && state_is_updated()))
        return;
    delete_line();
    _display->print("State: ");
    switch (get_state())
    {
    case MAV_STATE_UNINIT:
        _display->print("UNINIT");
        break;
    case MAV_STATE_BOOT:
        _display->print("BOOT");
        break;
    case MAV_STATE_CALIBRATING:
        _display->print("CALIB");
        break;
    case MAV_STATE_STANDBY:
        _display->setTextColor(DISPLAY_COLOR_DISARMED);
        _display->print("STANDBY");
        _display->setTextColor(DISPLAY_COLOR_NORMAL);
        // TODO: handle LEDS
        // if (screen_data.prearm_check_passed)
        // 	led_color = LED_READY_TO_FLY;
        // else
        // 	led_color = LED_NOT_READY;
        break;
    case MAV_STATE_ACTIVE:
        _display->setTextColor(DISPLAY_COLOR_ARMED);
        _display->print("ACTIVE");
        _display->setTextColor(DISPLAY_COLOR_NORMAL);
        // TODO: handle LEDS
        // led_color = LED_ARMED;
        break;
    case MAV_STATE_CRITICAL:
        _display->print("CRIT");
        break;
    case MAV_STATE_EMERGENCY:
        _display->print("EMERG");
        break;
    case MAV_STATE_POWEROFF:
        _display->print("OFF");
        break;
    case MAV_STATE_FLIGHT_TERMINATION:
        _display->print("TERMIN");
        break;
    default:
        _display->print("?");
        break;
    }
}

bool DisplayManager::is_armed()
{
    return get_state(true) == MAV_STATE_ACTIVE;
}
