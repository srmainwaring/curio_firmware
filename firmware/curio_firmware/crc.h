/*
 * CRC Calculation
 * 
 * Ralf Helbing
 * Technical Specification Document
 * HoTT SUMD Data Protocol, p6. 06/12/2012
 * https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
 * 
 */

#ifndef CURIO_FIRMWARE_CRC_H_
#define CURIO_FIRMWARE_CRC_H_

#include <inttypes.h>

/*******************************************************************************
 * Function Name : CRC16
 * Description : crc calculation, adds a 8 bit unsigned to 16 bit crc 
 *******************************************************************************/  
uint16_t CRC16(uint16_t crc, uint8_t value);

/*
 * CRC used by SUMD:
 * https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Math/crc.h
 * 
 * Modified for use with rosserial_arduino 2019 by Rhys Mainwaring
 */

#endif // CURIO_FIRMWARE_CRC_H_
