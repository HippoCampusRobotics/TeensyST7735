#include "px4_interface.h"
#include "led.h"

namespace px4_interface
{

    static DisplayManager *screen_data = NULL;

    static HardwareSerial *mavlink_port = NULL;

    void init(DisplayManager *ptr, HardwareSerial *port)
    {
        led_init();
        screen_data = ptr;
        mavlink_port = port;
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
        case MAVLINK_MSG_ID_COMMAND_ACK:
            handle_command_ack(msg);
            break;
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            handle_message_local_position_ned(msg);
            break;
        default:
            break;
        }
    }

    void handle_message_sys_status(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(msg, &sys_status);
        uint32_t now = millis();
        screen_data->update_voltage(sys_status.voltage_battery / 1000.0, now);
        screen_data->update_battery_remaining((int)sys_status.battery_remaining, now);
        bool ready_to_fly = (bool)(sys_status.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK);
        screen_data->update_prearm_check(ready_to_fly, now);
        if (!screen_data->is_armed())
        {
            if (ready_to_fly)
                led_set_state(LED_STATE_READY_TO_FLY);
            else
                led_set_state(LED_STATE_NOT_READY);
        }
    }

    void handle_message_attitude(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(msg, &attitude);
        uint32_t now = millis();
        screen_data->update_orientation((int)(attitude.roll * 180.0 / 3.14), (int)(attitude.pitch * 180.0 / 3.14), (int)(attitude.yaw * 180.0 / 3.14), now);
    }

    void handle_message_system_time(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_system_time_t system_time;
        mavlink_msg_system_time_decode(msg, &system_time);
        uint32_t now = millis();
        screen_data->update_time_boot_ms(system_time.time_boot_ms, now);
    }

    void handle_message_heartbeat(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(msg, &heartbeat);
        led_update_heartbeat();
        uint32_t now = millis();
        screen_data->update_mode(custom_mode(heartbeat.custom_mode), now);
        screen_data->update_state(heartbeat.system_status, now);
        if (screen_data->is_armed())
            led_set_state(LED_STATE_ARMED);
        else
        {
            if (screen_data->get_prearm_check(true))
                led_set_state(LED_STATE_READY_TO_FLY);
            else
                led_set_state(LED_STATE_NOT_READY);
        }
    }

    void handle_message_local_position_ned(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_local_position_ned_t pos;
        mavlink_msg_local_position_ned_decode(msg, &pos);
        uint32_t now = millis();
        // transform to ENU/ROS coordinates
        screen_data->update_position(pos.y, pos.x, -pos.z, now);
    }

    void handle_command_ack(mavlink_message_t *msg)
    {
        if (!screen_data)
            return;
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM)
        {
            if (ack.result != MAV_RESULT_ACCEPTED)
            {
                led_set_state(LED_STATE_ARMING_FAILED);
            }
        }
    }

    void send_heartbeat()
    {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(1, MAV_COMPONENT::MAV_COMP_ID_PERIPHERAL, &msg, MAV_TYPE::MAV_TYPE_GENERIC, MAV_AUTOPILOT::MAV_AUTOPILOT_GENERIC, MAV_MODE::MAV_MODE_MANUAL_DISARMED, 0, MAV_STATE::MAV_STATE_STANDBY);

        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
        if (mavlink_port)
        {
            mavlink_port->write(buffer, len);
        }
    }

}