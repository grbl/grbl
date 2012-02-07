// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/


unsigned char eeprom_get_char( unsigned int addr )
{
	return 0;
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
	return;
}

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) 
{
  
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size)
{
	return 0;
}

// end of file
