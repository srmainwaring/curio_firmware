/*
 * LOBOT SERIAL API
 * ----------------
 * 
 * We use the Lewansoul documentation and code from their DropBox area:
 * 
 * https://www.dropbox.com/sh/b3v81sb9nwir16q/AACkK-tg0q39fKJZcSl-YrqOa/LX-16A%20Bus%20Servo?dl=0&subfolder_nav_tracking=1
 * 
 * - Bus Servo Communication Routines
 *      - Use Arduino control servo
 *          - Read Angle
 *              - SerialServoRP.ino
 */

#ifndef CURIO_FIRMWARE_LOBOT_SERIAL_H_
#define CURIO_FIRMWARE_LOBOT_SERIAL_H_

#include <Arduino.h>
#include <HardwareSerial.h>

byte LobotCheckSum(byte buf[]);
void LobotSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);
void LobotSerialServoStopMove(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoAngleAdjust(HardwareSerial &SerialX, uint8_t id, uint8_t deviation);
void LobotSerialServoAngleWrite(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID);
void LobotSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed);
void LobotSerialServoLoad(HardwareSerial &SerialX, uint8_t id);
void LobotSerialServoUnload(HardwareSerial &SerialX, uint8_t id);
int LobotSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret);
int LobotSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id);
int LobotSerialServoReadVin(HardwareSerial &SerialX, uint8_t id);

#endif // CURIO_FIRMWARE_LOBOT_SERIAL_H_
