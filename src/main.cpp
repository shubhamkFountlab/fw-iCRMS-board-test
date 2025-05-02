#include <Arduino.h>
#include "global.h"
#include "modbus.h"

CRGB leds[NUM_LEDS];
uint16_t rOP = 0;
void setup()
{

  Serial.begin(115200);
  FastLED.addLeds<WS2812B, RGB_LED_PIN, GRB>(leds, NUM_LEDS);
  InitRtc();
  EthernetBegin();
  Wire.begin(SDA1_PIN, SCL1_PIN);  // I2C 1
  Wire1.begin(SDA2_PIN, SCL2_PIN); // I2C 2
  PCAL6532DigitalIOPinModesInit();
  ESP32DigitalIPPinModesInit();
  AdcInit();
  DacInit();
  pinMode(HEARTBEAT_LED, OUTPUT);
  digitalWrite(HEARTBEAT_LED, LOW); // Set the heartbeat LED to LOW initially
  modbusInit();                     // Initialize Modbus
}

void loop()
{

  static uint32_t _hbLastMillis = 0, _opLastMillis = 0;
  if (millis() - _hbLastMillis > 5000)
  {
    _hbLastMillis = millis();
    digitalWrite(HEARTBEAT_LED, !digitalRead(HEARTBEAT_LED));
    GetDigitalInputRegisters();
    GetAnalogInputRegisters();
    if (rOP == 0)
    {
      coil_reg.coil = 0x0000;
    }
    else
    {
      coil_reg.coil = 0xFFFF;
    }
    rOP = !rOP;
    // debugPrintln("Heartbeat");
  }
  // RGB LED test
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  delay(100);
  leds[0] = CRGB(150, 0, 255);
  FastLED.show();
  SendDigitalOutputToRegisters();
  SendAnalogOutputToRegisters(rOP);
}
