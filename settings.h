#ifndef __SETTINGS_H__
#define __SETTINGS_H__

struct settings_V1 {
    uint8_t nodeid;
    uint8_t gatewayid;
    uint8_t networkid;
    uint8_t radioFrequency;
    uint8_t radioPowerLevel;
    bool    radioIsHighPower;
    bool    bme280Pressure;
    int16_t wakeupsPerHour;
};

#endif
