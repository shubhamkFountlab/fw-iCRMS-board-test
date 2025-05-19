#include <Arduino.h>
#include <FastLED.h>
#include <MCP7940.h>
#include "SPI.h"

#define RGB_LED_PIN 13
#define NUM_LEDS 1

// I2C 1 Pins
#define SDA1_PIN 5
#define SCL1_PIN 4
// I2C 2 Pins
#define SDA2_PIN 33
#define SCL2_PIN 32

// Digital Input Pins on ESP32
#define DIGITAL_IP1 36
#define DIGITAL_IP2 39
#define DIGITAL_IP3 34
#define DIGITAL_IP4 35

#define HEARTBEAT_LED 0 /// GPIO0

// Ethernet Pins
#define ETH_CLK_MODE ETH_CLOCK_GPIO0_IN // ETH_CLOCK_GPIO17_OUT
#define ETH_POWER_PIN -1                // The enable signal for the external crystal oscillator (-1 to disable for internal APLL source)
#define ETH_TYPE ETH_PHY_LAN8720        // Type of the Ethernet PHY LAN8720
#define ETH_ADDR 1                      // I²C-address of Ethernet PHY
#define ETH_MDC_PIN 23                  // The I²C clock signal for the Ethernet PHY
#define ETH_MDIO_PIN 18                 // The I²C IO signal for the Ethernet PHY
#define NRST 21                         // The nRST (enable) for the Ethernet PHY


typedef union _COIL_REG1_
{
    uint16_t coil;
    struct
    {
        bool digitalOp1 : 1;  // DO 1
        bool digitalOp2 : 1;  // DO 2
        bool digitalOp3 : 1;  // DO 3
        bool digitalOp4 : 1;  // DO 4
        bool digitalOp5 : 1;  // DO 5
        bool digitalOp6 : 1;  // DO 6
        bool digitalOp7 : 1;  // DO 7
        bool digitalOp8 : 1;  // DO 8
        bool digitalOp9 : 1;  // DO 9
        bool digitalOp10 : 1; // DO 10
        bool digitalOp11 : 1; // DO 11
        bool digitalOp12 : 1; // DO 12
    };
} COIL_REG1;

extern COIL_REG1 coil_reg; // Global variable to pull status of coils.

void InitRtc();
void getDateTime();
void EthernetBegin();
void PCAL6532DigitalIOPinModesInit();
void ESP32DigitalIPPinModesInit();
void SendDigitalOutputToRegisters(uint8_t i);
void GetDigitalInputRegisters();
void GetAnalogInputRegisters();
void AdcInit();
void SendAnalogOutputToRegisters(uint16_t val);
void DacInit();
void eepromPageWrite(uint16_t addr, byte *data, uint8_t length);
void eepromPageRead(uint16_t addr, byte *buffer, uint8_t length);