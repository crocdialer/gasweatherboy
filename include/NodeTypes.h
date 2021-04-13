#ifndef _NODE_TYPES_H
#define _NODE_TYPES_H

#include <stdint.h>

#define STRUCT_TYPE_GASWEATHERBOY 0xF1

typedef struct gasweatherboy_t
{
    uint8_t stype = STRUCT_TYPE_GASWEATHERBOY;

    // battery in range [0 .. 1]
    uint8_t battery = 0;

    //! temperature in range [-50C .. 100C]
    uint16_t temperature = 0;

    //! pressure in range [0hPa .. 2000hPa]
    uint16_t pressure = 0;

    // relative humidity in range [0..1]
    uint8_t humidity = 0;

    // eCO2 (equivalent calculated carbon-dioxide) concentration in range [400 .. 8192] parts per million (ppm)
    uint16_t eco2 = 0;

    // TVOC (total volatile organic compound) concentration in range [0 .. 1187] parts per billion (ppb)
    uint16_t tvoc = 0;

} gasweatherboy_t;

#endif
