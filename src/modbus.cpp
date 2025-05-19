#include "modbus.h"

ModbusServerRTU MBserver1(timeOut, RS485_1_DE_PIN);
ModbusServerRTU MBserver2(timeOut, RS485_2_DE_PIN); // RS485 2

void modbusInit()
{
    // RS485 1
    RTUutils::prepareHardwareSerial(Serial1);
    Serial1.begin(RS485_1_BAUD, SERIAL_8N1, RS485_1_RX_PIN, RS485_1_TX_PIN);
    MBserver1.begin(Serial1);
    // Packet Length error  solve using below function
    MBserver1.skipLeading0x00();
    MBserver1.registerWorker(slaveID, READ_HOLD_REGISTER, &FC03HoldingsRegisters);

    // RS485 2
    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(RS485_2_BAUD, SERIAL_8N1, RS485_2_RX_PIN, RS485_2_TX_PIN);
    MBserver2.begin(Serial2);
    // Packet Length error  solve using below function
    MBserver2.skipLeading0x00();
    MBserver2.registerWorker(slaveID2, READ_HOLD_REGISTER, &FC03HoldingsRegisters1);
    Serial.println("Modbus Server Initialized");    
}
// FC03: worker do serve Modbus function code 0x03 (READ_HOLD_REGISTER)
ModbusMessage FC03HoldingsRegisters(ModbusMessage request)
{

    uint16_t startAddress;  // requested register startAddress
    uint16_t words;         // requested number of registers
    ModbusMessage response; // response message to be sent back

    // get request values
    request.get(2, startAddress);
    request.get(4, words);
    Serial.print("MODBUS RX: ");
    Serial.printf("%02X %02X %02X %02X\n", request.getServerID(), request.getFunctionCode(), startAddress, words);
    // Address and words valid? We assume 10 registers here for demo
    if ((startAddress >= 0) && ((startAddress + words) <= 4))
    {
        // Looks okay. Set up message with serverID, FC and length of data
        response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
        // Fill response with requested data
        for (uint16_t address = startAddress; address < startAddress + words; ++address)
        {
            if (address == 0)
            {
                response.add(uint16_t(222));
            }
            else if (address == 1)
            {
                response.add(uint16_t(333));
            }
            else if (address == 2)
            {
                response.add(uint16_t(444));
            }
            else if (address == 3)
            {
                response.add(uint16_t(555));
            }
        }
    }
    else
    {
        // No, either startAddress or words are outside the limits. Set up error response.
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }
    Serial.print("MODBUS TX: ");
    Serial.printf("%02X %02X %02X %02X", request.getServerID(), request.getFunctionCode(), (words * 2));
    Serial.printf(" %02X %02X %02X %02X\n", 222, 333, 444, 555);
    return response;
}

// FC03: worker do serve Modbus function code 0x03 (READ_HOLD_REGISTER)
ModbusMessage FC03HoldingsRegisters1(ModbusMessage request)
{

    uint16_t startAddress;  // requested register startAddress
    uint16_t words;         // requested number of registers
    ModbusMessage response; // response message to be sent back

    // get request values
    request.get(2, startAddress);
    request.get(4, words);
    Serial.print("MODBUS RX: ");
    Serial.printf("%02X %02X %02X %02X\n", request.getServerID(), request.getFunctionCode(), startAddress, words);
    // Address and words valid? We assume 10 registers here for demo
    if ((startAddress >= 11) && ((startAddress + words) <= 15))
    {
        // Looks okay. Set up message with serverID, FC and length of data
        response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
        // Fill response with requested data
        for (uint16_t address = startAddress; address < startAddress + words; ++address)
        {
            if (address == 11)
            {
                response.add(uint16_t(234));
            }
            else if (address == 12)
            {
                response.add(uint16_t(987));
            }
            else if (address == 13)
            {
                response.add(uint16_t(212));
            }
            else if (address == 14)
            {
                response.add(uint16_t(454));
            }
        }
    }
    else
    {
        // No, either startAddress or words are outside the limits. Set up error response.
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    }
    Serial.print("MODBUS TX: ");
    Serial.printf("%02X %02X %02X %02X", request.getServerID(), request.getFunctionCode(), (words * 2));
    Serial.printf(" %02X %02X %02X %02X\n", 234, 987, 212, 454);
    return response;
}