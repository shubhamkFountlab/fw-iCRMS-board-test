#include <Arduino.h>
#include "global.h"
#include "modbus.h"

// CRGB leds[NUM_LEDS];
uint16_t rOP = 0;
int rl = 0;
void setup()
{

  Serial.begin(115200);
  Wire.begin(SDA1_PIN, SCL1_PIN); // I2C 1
  InitRtc();
  EthernetBegin();
  PCAL6532DigitalIOPinModesInit();
  ESP32DigitalIPPinModesInit();
  Wire1.begin(SDA2_PIN, SCL2_PIN); // I2C 2
  AdcInit();
  DacInit();
  pinMode(RGB_LED_PIN, INPUT); // Set RGB LED pin as input
  // pinMode(HEARTBEAT_LED, OUTPUT);
  // digitalWrite(HEARTBEAT_LED, LOW); // Set the heartbeat LED to LOW initially
  modbusInit(); // Initialize Modbus
  // FastLED.addLeds<WS2812B, RGB_LED_PIN, GRB>(leds, NUM_LEDS);
  byte dataToWrite[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; // 16 bytes
  byte s[8] = {'S', 'H', 'U', 'B', 'H', 'A', 'M'};
  uint16_t startAddress = 0x0001;

  eepromPageWrite(startAddress, dataToWrite, 16);
  delay(20);                                // wait to complete writing
  eepromPageWrite(startAddress + 16, s, 8); // Write 8 bytes to the next page

  delay(20); // wait to complete writing

  byte dataRead[16];
  byte sRead[8];
  eepromPageRead(startAddress, dataRead, 16);
  eepromPageRead(startAddress + 16, sRead, 8); // Read 8 bytes from the next page

  Serial.println("Data read from EEPROM:");
  for (int i = 0; i < 16; i++)
  {
    Serial.print(dataRead[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("String read from EEPROM: ");
  for (int i = 0; i < 8; i++)
  {
    Serial.print((char)sRead[i]);
  }
  Serial.println();
}

void loop()
{

  static uint32_t _hbLastMillis = 0, _opLastMillis = 0;
  static unsigned long _rlMillis = 0;
  if (millis() - _hbLastMillis > 5000)
  {
    _hbLastMillis = millis();
    // digitalWrite(HEARTBEAT_LED, !digitalRead(HEARTBEAT_LED));
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
  // leds[0] = CRGB(255, 0, 0);
  // FastLED.show();
  // delay(100);
  // leds[0] = CRGB(0, 255, 0);
  // FastLED.show();
  // delay(100);
  // leds[0] = CRGB(0, 0, 255);
  // FastLED.show();
  // delay(100);
  // leds[0] = CRGB(150, 0, 255);
  // FastLED.show();

  // // Relay Start One by One
  // if (millis() - _rlMillis > 2000)
  // {
  //   rl = rl + 1;
  //   SendDigitalOutputToRegisters(rl);
  //   _rlMillis = millis();
  //   if (rl == 12)
  //   {
  //     rl = 0;
  //   }
  // }

  SendDigitalOutputToRegisters(rl);
  SendAnalogOutputToRegisters(rOP);
}
