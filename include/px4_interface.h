#ifndef __px4_interface_H_
#define __px4_interface_H_

#include "standard/mavlink.h"
#include "display.h"
namespace px4_interface
{
   

    void handle_mavlink_message(mavlink_message_t *msg);
    void handle_message_sys_status(mavlink_message_t *msg);
    void handle_message_attitude(mavlink_message_t *msg);
    void handle_message_system_time(mavlink_message_t *msg);
    void handle_message_heartbeat(mavlink_message_t *msg);
    void handle_command_ack(mavlink_message_t *msg);
    void handle_message_local_position_ned(mavlink_message_t *msg);
    void set_display_manager(DisplayManager *ptr);
    void set_mavlink_port(HardwareSerial *port);
    void send_heartbeat();
    void init(DisplayManager *ptr, HardwareSerial *port);
}
#endif