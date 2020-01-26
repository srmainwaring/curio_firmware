/*
 * CRC Calculation
 * 
 * Ralf Helbing
 * Technical Specification Document
 * HoTT SUMD Data Protocol, p6. 06/12/2012
 * https://www.deviationtx.com/media/kunena/attachments/98/HoTT-SUMD-Spec-REV01-12062012-pdf.pdf
 * 
 */

#include "crc.h"

#define CRC_POLYNOME 0x1021

/*******************************************************************************
 * Function Name : CRC16
 * Description : crc calculation, adds a 8 bit unsigned to 16 bit crc 
 *******************************************************************************/  
uint16_t CRC16(uint16_t crc, uint8_t value)
{
  uint8_t i;
  crc = crc ^ (int16_t)value<<8;
  for(i=0; i<8; i++)
  {
    if (crc & 0x8000)
      crc = (crc << 1) ^ CRC_POLYNOME;
    else
      crc = (crc << 1);
  }
  return crc;
}
