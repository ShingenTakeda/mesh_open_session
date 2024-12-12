#pragma once
#include <stdint.h>

typedef struct{
    uint8_t device_id[3];
    uint8_t mac[6];
}MeterAttrib;

extern MeterAttrib meter_attrib;