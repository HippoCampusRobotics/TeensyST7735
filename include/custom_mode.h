#ifndef __custom_mode_H_
#define __custom_mode_H_
#include <stdint.h>
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
#endif