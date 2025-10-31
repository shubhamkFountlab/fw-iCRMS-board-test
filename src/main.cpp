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
                                  // ...existing code...
  uint64_t mac = ESP.getEfuseMac();
  Serial.print("ESP32 MAC Address: ");
  Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
                (uint8_t)(mac >> 40),
                (uint8_t)(mac >> 32),
                (uint8_t)(mac >> 24),
                (uint8_t)(mac >> 16),
                (uint8_t)(mac >> 8),
                (uint8_t)(mac));
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

  // Test struct
  tCOMP_RUN_HOURS_MILLIS runData = {
      .compRunHoursStartMillis = 111111,
      .ccsv1RunHoursStartMillis = 222222,
      .ccsv2RunHoursStartMillis = 333333,
      .ccsv3RunHoursStartMillis = 444444,
      .ccsv4RunHoursStartMillis = 555555,
      .crc16 = 0};

  writeStructWithCRC(0x0000, &runData);
  delay(20); // Wait EEPROM write cycle to complete
  Serial.println("Data written to EEPROM");

  tCOMP_RUN_HOURS_MILLIS readBack;
  if (readStructWithCRC(0x0000, &readBack))
  {
    Serial.println("EEPROM data is valid:");
    Serial.println(readBack.compRunHoursStartMillis);
    Serial.println(readBack.ccsv1RunHoursStartMillis);
    Serial.println(readBack.ccsv2RunHoursStartMillis);
    Serial.println(readBack.ccsv3RunHoursStartMillis);
    Serial.println(readBack.ccsv4RunHoursStartMillis);
    Serial.print("CRC: 0x");
    Serial.println(readBack.crc16, HEX);
  }
  else
  {
    Serial.println("EEPROM data is corrupted!");
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
    getDateTime();
    if (rOP == 0)
    {
      coil_reg.coil = 0x0000;
    }
    else
    {
      coil_reg.coil = 0xFFFF;
    }
    rOP = !rOP;
    SendDigitalOutputToRegisters(rl);
    SendAnalogOutputToRegisters(rOP);
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
}
