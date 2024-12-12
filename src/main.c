#include <stdio.h>

#include "mesh.h"
#include "uart.h"
#include "common.h"

#define ROUTER_SSID "AndroidAP2CC1" // WiFi network's name
#define ROUTER_PWD "rzue0381"   // WiFi network's Password
#define MESH_AP_PASS "meterfarm1" // MESH network's password

uint8_t my_mac[6];

MeterAttrib meter_attrib;

static uint8_t MESH_ID[6] = { 0x12, 0x34, 0x56, 0x78, 0x90, 0x91 }; // MESH network MAC address (mesh ID)

void app_main(void)
{
    
    start_uart();
    start_mesh(ROUTER_SSID, ROUTER_PWD, MESH_ID, MESH_AP_PASS);
}