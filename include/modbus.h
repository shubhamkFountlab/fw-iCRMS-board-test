
#include "ModbusServerRTU.h"
#include "HardwareSerial.h"

#define timeOut 2000
// RS485 1  Pins
#define RS485_1_TX_PIN 15
#define RS485_1_RX_PIN 14
#define RS485_1_DE_PIN 12
#define RS485_1_BAUD 9600
#define RS485_1_DATA_BITS 8
#define RS485_1_PARITY_BITS 0
#define slaveID 1 // Modbus Slave ID
// RS485 2  Pins
#define RS485_2_TX_PIN 17
#define RS485_2_RX_PIN 16
#define RS485_2_DE_PIN 2
#define RS485_2_BAUD 19200
#define RS485_2_DATA_BITS 8
#define RS485_2_PARITY_BITS 0
#define slaveID2 2 // Modbus Slave ID 2

ModbusMessage FC03HoldingsRegisters(ModbusMessage request);
ModbusMessage FC03HoldingsRegisters1(ModbusMessage request);

void modbusInit();