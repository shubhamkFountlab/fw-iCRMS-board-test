#include "global.h"
#include <Wire.h>
#include <PCAL6524.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4728.h>
#include <ETH.h>

MCP7940_Class MCP7940;
PCAL6524 ioPCAL6524;
COIL_REG1 coil_reg;
Adafruit_ADS1115 adcCH1; // one channel have only 4 adc input
Adafruit_ADS1115 adcCH2;
Adafruit_MCP4728 dac1; // MCP4728 4-Channel 12-bit I2C DAC

char ipAddr[20], subnet[20], gateway[20];
IPAddress local_IP, gateway_IP, subnet_Mask;

static bool eth_connected = false; // Global variable to pull status of eth.

void InitRtc()
{
    Serial.print(F("\nStarting SetAndCalibrate program\n"));
    Serial.print(F("- Compiled with c++ version "));
    Serial.print(F(__VERSION__)); // Show compiler information
    Serial.print(F("\n- On "));
    Serial.print(F(__DATE__));
    Serial.print(F(" at "));
    Serial.print(F(__TIME__));
    Serial.print(F("\n"));
    while (!MCP7940.begin(SDA1_PIN, SCL1_PIN))
    {
        Serial.println("Unable to find RTC");
        delay(1000);
    }
    Serial.println("RTC Initialized!");
    MCP7940.adjust(DateTime(F(__DATE__), F(__TIME__)));
}
void getDateTime()
{
    DateTime now = MCP7940.now();
    Serial.printf("%04d-%02d-%02d %02d:%02d:%02d",
                  now.year(), // Use sprintf() to pretty print
                  now.month(), now.day(), now.hour(), now.minute(),
                  now.second());
    Serial.println();
}

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case ARDUINO_EVENT_ETH_START:
        Serial.println("ETH Started");
        // set eth hostname here
        ETH.setHostname("modbus-tcp");
        break;
    case ARDUINO_EVENT_ETH_CONNECTED:
        Serial.println("ETH Connected");
        break;
    case ARDUINO_EVENT_ETH_GOT_IP:
        Serial.print("ETH MAC: ");
        Serial.println(ETH.macAddress());
        Serial.print(", IPv4: ");
        Serial.println(ETH.localIP());
        if (ETH.fullDuplex())
        {
            Serial.println(", FULL_DUPLEX");
        }
        Serial.println(", ");
        Serial.println(ETH.linkSpeed());
        Serial.println("Mbps");
        strcpy(subnet, ETH.subnetMask().toString().c_str());
        strcpy(ipAddr, ETH.localIP().toString().c_str());
        strcpy(gateway, ETH.gatewayIP().toString().c_str());
        break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
        Serial.println("ETH Disconnected");
        break;
    case ARDUINO_EVENT_ETH_STOP:
        Serial.println("ETH Stopped");
        break;
    default:
        break;
    }
}

void EthernetBegin()
{
    WiFi.onEvent(WiFiEvent);

    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
    Serial.print("Ethernet Connected IP :");
    Serial.println(ETH.localIP());
}

void PCAL6532DigitalIOPinModesInit()
{

    ioPCAL6524.begin();

    // Digital Output Pins
    ioPCAL6524.remotedigitalWrite(P1_4, LOW); // DO 1
    ioPCAL6524.remotedigitalWrite(P1_5, LOW); // DO 2
    ioPCAL6524.remotedigitalWrite(P1_6, LOW); // DO 3
    ioPCAL6524.remotedigitalWrite(P1_7, LOW); // DO 4
    ioPCAL6524.remotedigitalWrite(P2_0, LOW); // DO 5
    ioPCAL6524.remotedigitalWrite(P2_1, LOW); // DO 6
    ioPCAL6524.remotedigitalWrite(P2_2, LOW); // DO 7
    ioPCAL6524.remotedigitalWrite(P2_3, LOW); // DO 8
    ioPCAL6524.remotedigitalWrite(P2_4, LOW); // DO 9
    ioPCAL6524.remotedigitalWrite(P2_5, LOW); // DO 10
    ioPCAL6524.remotedigitalWrite(P2_6, LOW); // DO 11
    ioPCAL6524.remotedigitalWrite(P2_7, LOW); // DO 12

    ioPCAL6524.remotepinMode(P1_4, OUTPUT); // DO 1
    ioPCAL6524.remotepinMode(P1_5, OUTPUT); // DO 2
    ioPCAL6524.remotepinMode(P1_6, OUTPUT); // DO 3
    ioPCAL6524.remotepinMode(P1_7, OUTPUT); // DO 4
    ioPCAL6524.remotepinMode(P2_0, OUTPUT); // DO 5
    ioPCAL6524.remotepinMode(P2_1, OUTPUT); // DO 6
    ioPCAL6524.remotepinMode(P2_2, OUTPUT); // DO 7
    ioPCAL6524.remotepinMode(P2_3, OUTPUT); // DO 8
    ioPCAL6524.remotepinMode(P2_4, OUTPUT); // DO 9
    ioPCAL6524.remotepinMode(P2_5, OUTPUT); // DO 10
    ioPCAL6524.remotepinMode(P2_6, OUTPUT); // DO 11
    ioPCAL6524.remotepinMode(P2_7, OUTPUT); // DO 12

    // Digital Input Pins
    ioPCAL6524.remotepinMode(P0_0, INPUT); // DI 5
    ioPCAL6524.remotepinMode(P0_1, INPUT); // DI 6
    ioPCAL6524.remotepinMode(P0_2, INPUT); // DI 7
    ioPCAL6524.remotepinMode(P0_3, INPUT); // DI 8
    ioPCAL6524.remotepinMode(P0_4, INPUT); // DI 9
    ioPCAL6524.remotepinMode(P0_5, INPUT); // DI 10
    ioPCAL6524.remotepinMode(P0_6, INPUT); // DI 11
    ioPCAL6524.remotepinMode(P0_7, INPUT); // DI 12
    ioPCAL6524.remotepinMode(P1_0, INPUT); // DI 13
    ioPCAL6524.remotepinMode(P1_1, INPUT); // DI 14
    ioPCAL6524.remotepinMode(P1_2, INPUT); // DI 15
    ioPCAL6524.remotepinMode(P1_3, INPUT); // DI 16

    Serial.println("PCAL6524 Digital Input/Output Pinmodes Init");
}

void ESP32DigitalIPPinModesInit()
{
    pinMode(DIGITAL_IP1, INPUT); // DI 1
    pinMode(DIGITAL_IP2, INPUT); // DI 2
    pinMode(DIGITAL_IP3, INPUT); // DI 3
    pinMode(DIGITAL_IP4, INPUT); // DI 4
    Serial.println("ESP32 Digital Input Pinmodes Init");
}

void GetDigitalInputRegisters()
{
    Serial.printf("IP1: %d\n", !digitalRead(DIGITAL_IP1));
    Serial.printf("IP2: %d\n", !digitalRead(DIGITAL_IP2));
    Serial.printf("IP3: %d\n", !digitalRead(DIGITAL_IP3));
    Serial.printf("IP4: %d\n", !digitalRead(DIGITAL_IP4));
    Serial.printf("IP5: %d\n", !ioPCAL6524.remotedigitalRead(P0_0));
    Serial.printf("IP6: %d\n", !ioPCAL6524.remotedigitalRead(P0_1));
    Serial.printf("IP7: %d\n", !ioPCAL6524.remotedigitalRead(P0_2));
    Serial.printf("IP8: %d\n", !ioPCAL6524.remotedigitalRead(P0_3));
    Serial.printf("IP9: %d\n", !ioPCAL6524.remotedigitalRead(P0_4));
    Serial.printf("IP10: %d\n", !ioPCAL6524.remotedigitalRead(P0_5));
    Serial.printf("IP11: %d\n", !ioPCAL6524.remotedigitalRead(P0_6));
    Serial.printf("IP12: %d\n", !ioPCAL6524.remotedigitalRead(P0_7));
    Serial.printf("IP13: %d\n", !ioPCAL6524.remotedigitalRead(P1_0));
    Serial.printf("IP14: %d\n", !ioPCAL6524.remotedigitalRead(P1_1));
    Serial.printf("IP15: %d\n", !ioPCAL6524.remotedigitalRead(P1_2));
    Serial.printf("IP16: %d\n", !ioPCAL6524.remotedigitalRead(P1_3));
}

void SendDigitalOutputToRegisters()
{
    ioPCAL6524.remotedigitalWrite(P1_4, coil_reg.digitalOp1);  // DO 1
    ioPCAL6524.remotedigitalWrite(P1_5, coil_reg.digitalOp2);  // DO 2
    ioPCAL6524.remotedigitalWrite(P1_6, coil_reg.digitalOp3);  // DO 3
    ioPCAL6524.remotedigitalWrite(P1_7, coil_reg.digitalOp4);  // DO 4
    ioPCAL6524.remotedigitalWrite(P2_0, coil_reg.digitalOp5);  // DO 5
    ioPCAL6524.remotedigitalWrite(P2_1, coil_reg.digitalOp6);  // DO 6
    ioPCAL6524.remotedigitalWrite(P2_2, coil_reg.digitalOp7);  // DO 7
    ioPCAL6524.remotedigitalWrite(P2_3, coil_reg.digitalOp8);  // DO 8
    ioPCAL6524.remotedigitalWrite(P2_4, coil_reg.digitalOp9);  // DO 9
    ioPCAL6524.remotedigitalWrite(P2_5, coil_reg.digitalOp10); // DO 10
    ioPCAL6524.remotedigitalWrite(P2_6, coil_reg.digitalOp11); // DO 11
    ioPCAL6524.remotedigitalWrite(P2_7, coil_reg.digitalOp12); // DO 12
}

void AdcInit()
{
    Wire1.begin(SDA2_PIN, SCL2_PIN);
    adcCH1.setGain(GAIN_ONE);
    adcCH2.setGain(GAIN_ONE);
    adcCH1.begin(0x48, &Wire1);
    adcCH2.begin(0x49, &Wire1);
    Serial.println("ADC Initialized");
}

void GetAnalogInputRegisters()
{
    Serial.printf("ADC1: %d\n", adcCH1.readADC_SingleEnded(2));
    Serial.printf("ADC2: %d\n", adcCH1.readADC_SingleEnded(1));
    Serial.printf("ADC3: %d\n", adcCH1.readADC_SingleEnded(0));
    Serial.printf("ADC4: %d\n", adcCH1.readADC_SingleEnded(3));
    Serial.printf("ADC5: %d\n", adcCH2.readADC_SingleEnded(2));
    Serial.printf("ADC6: %d\n", adcCH2.readADC_SingleEnded(1));
    Serial.printf("ADC7: %d\n", adcCH2.readADC_SingleEnded(0));
    Serial.printf("ADC8: %d\n", adcCH2.readADC_SingleEnded(3));
}

void DacInit()
{
    // Wire.begin(SDA1_PIN, SCL1_PIN);
    if (!dac1.begin(0x60), &Wire)
    {
        Serial.println("Failed to find MCP4728 DAC1 chip");
    }
}

void SendAnalogOutputToRegisters(uint16_t val)
{
    if (val == 1)
    {
        dac1.setChannelValue(MCP4728_CHANNEL_A, 1024, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_B, 2048, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_C, 3072, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_D, 4094, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
    }
    else if (val == 2)
    {
        dac1.setChannelValue(MCP4728_CHANNEL_A, 4094, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_B, 3072, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_C, 2048, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
        dac1.setChannelValue(MCP4728_CHANNEL_D, 1024, MCP4728_VREF_INTERNAL, MCP4728_GAIN_1X);
    }
}


#define EEPROM_I2C_ADDRESS 0x50 // Base address for AT24LC16 (A0, A1, A2 = 0)

void eepromPageWrite(uint16_t addr, byte *data, uint8_t length)
{
  if (length > 16)
    length = 16; // AT24LC16 page size is 16 bytes

  Wire.beginTransmission(EEPROM_I2C_ADDRESS | ((addr >> 8) & 0x07));
  Wire.write((byte)(addr & 0xFF)); // lower 8 bits
  for (uint8_t i = 0; i < length; i++)
  {
    Wire.write(data[i]);
  }
  Wire.endTransmission();

  delay(10); // wait EEPROM internal write cycle (typical 5ms-10ms)
  Serial.println("EEPROM Write Done!");
}
void eepromPageRead(uint16_t addr, byte *buffer, uint8_t length)
{
  if (length > 16)
    length = 16; // Read maximum 16 bytes at once

  Wire.beginTransmission(EEPROM_I2C_ADDRESS | ((addr >> 8) & 0x07));
  Wire.write((byte)(addr & 0xFF)); // lower 8 bits
  Wire.endTransmission();

  Wire.requestFrom((EEPROM_I2C_ADDRESS | ((addr >> 8) & 0x07)), length);
  uint8_t i = 0;
  while (Wire.available() && i < length)
  {
    buffer[i++] = Wire.read();
  }
}