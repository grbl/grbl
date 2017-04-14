/* 
* File:   ramps.h
* Author: arsi
* Enchanced by: Per Ivar Nerseth perivar@nerseth.com
*
* Created on September 1, 2014, 4:44 PM - arsi
* Cleanup and enhanced to support both soft and hard limits, March 2016 - perivar
*/
#ifndef RAMPS_H
#define	RAMPS_H

// Used to debug the limit switches
//#define RAMPS_DEBUG_LIMIT_SWITCHES

#ifdef	__cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include "cpu_map.h"
#include <stdint.h>
#include "fastio.h"

#define CHECK(var,pos) ((var) & (1<<(pos)))

	/**
	* Initialize the steppers (Step, Dir and Enable Pins)
	*/
	inline void rampsInitSteppers() {

    // Initialize Step Pins
    #if defined(X_STEP_PIN) && X_STEP_PIN > -1
      SET_OUTPUT(X_STEP_PIN);
    #endif
    #if defined(Y_STEP_PIN) && Y_STEP_PIN > -1
      SET_OUTPUT(Y_STEP_PIN);
    #endif
    #if defined(Z_STEP_PIN) && Z_STEP_PIN > -1
      SET_OUTPUT(Z_STEP_PIN);
    #endif
      
    // Initialize Dir Pins
    #if defined(X_DIR_PIN) && X_DIR_PIN > -1
      SET_OUTPUT(X_DIR_PIN);
    #endif
    #if defined(Y_DIR_PIN) && Y_DIR_PIN > -1
      SET_OUTPUT(Y_DIR_PIN);
    #endif
    #if defined(Z_DIR_PIN) && Z_DIR_PIN > -1
      SET_OUTPUT(Z_DIR_PIN);
    #endif
   
    // Initialize Enable Pins
    #if defined(X_ENABLE_PIN) && X_ENABLE_PIN > -1
      SET_OUTPUT(X_ENABLE_PIN);
    #endif
    #if defined(Y_ENABLE_PIN) && Y_ENABLE_PIN > -1
      SET_OUTPUT(Y_ENABLE_PIN);
    #endif
    #if defined(Z_ENABLE_PIN) && Z_ENABLE_PIN > -1
      SET_OUTPUT(Z_ENABLE_PIN);
    #endif
	}

	/**
	* Set the Enable Pin signal (either to high or low)
	* @param value
	*/
	inline void rampsWriteDisable(uint8_t value) {
		if (CHECK(value, STEPPERS_DISABLE_BIT)) {
			WRITE(X_ENABLE_PIN, HIGH);
			WRITE(Y_ENABLE_PIN, HIGH);
			WRITE(Z_ENABLE_PIN, HIGH);
		} else {
			WRITE(X_ENABLE_PIN, LOW);
			WRITE(Y_ENABLE_PIN, LOW);
			WRITE(Z_ENABLE_PIN, LOW);
		}
	}

	/**
	* Write stepper pulse (Step Pins)
	* @param value
	*/
	inline void rampsWriteSteps(uint8_t value) {
		if (CHECK(value, X_STEP_BIT)) {
			WRITE(X_STEP_PIN, HIGH);
		} else {
			WRITE(X_STEP_PIN, LOW);
		}
		if (CHECK(value, Y_STEP_BIT)) {
			WRITE(Y_STEP_PIN, HIGH);
		} else {
			WRITE(Y_STEP_PIN, LOW);
		}
		if (CHECK(value, Z_STEP_BIT)) {
			WRITE(Z_STEP_PIN, HIGH);
		} else {
			WRITE(Z_STEP_PIN, LOW);
		}
	}

	/**
  * Write stepper direction (Direction Pins)
	* @param value
	*/
	inline void rampsWriteDirections(uint8_t value) {
		if (CHECK(value, X_DIRECTION_BIT)) {
			WRITE(X_DIR_PIN, HIGH);
		} else {
			WRITE(X_DIR_PIN, LOW);
		}
		if (CHECK(value, Y_DIRECTION_BIT)) {
			WRITE(Y_DIR_PIN, HIGH);
		} else {
			WRITE(Y_DIR_PIN, LOW);
		}
		if (CHECK(value, Z_DIRECTION_BIT)) {
			WRITE(Z_DIR_PIN, LOW);
		} else {
			WRITE(Z_DIR_PIN, HIGH);
		}
	}

  /**
  * Enable interrupts for the limit switches
  */ 
  inline void rampsLimitsEnableInterrups() {
    
    // First enable the External Interrupts
    // When changing the ISCn bit, an interrupt can occur. Therefore, it is recommended to:
    // 1. First disable INTn by clearing its Interrupt Enable bit in the EIMSK Register. 
    // 2. Then, the ISCn bit can be changed. 
    // 3. Finally, the INTn interrupt flag should be cleared by writing a logical one to its Interrupt Flag bit (INTFn) in the EIFR Register 
    // 4. Before the interrupt is re-enabled.

    // X_MIN_PIN uses External Interrupt
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1    
      EIMSK &= ~(1 << X_MIN_INT);       // Disable external interrupt INT5 to ensure no interrupts are generated
      X_MIN_ICR &= ~(1 << X_MIN_ISCX1); // Set INT5 to trigger on ANY logic change (bit 1 = 0)
      X_MIN_ICR |=  (1 << X_MIN_ISCX0); // Set INT5 to trigger on ANY logic change (bit 0 = 1)      
      EIFR = (1 << X_MIN_FR);           // Clear flag for INT5 (by writing a logical one) to clear any pending interrupts
      EIMSK |= (1 << X_MIN_INT);        // Enable external interrupt INT5
    #endif

    // Z_MAX_PIN uses External Interrupt
    #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1    
      EIMSK &= ~(1 << Z_MAX_INT);       // Disable external interrupt INT2 to ensure no interrupts are generated
      Z_MAX_ICR &= ~(1 << Z_MAX_ISCX1); // Set INT2 to trigger on ANY logic change (bit 1 = 0)
      Z_MAX_ICR |=  (1 << Z_MAX_ISCX0); // Set INT2 to trigger on ANY logic change (bit 0 = 1)
      EIFR = (1 << Z_MAX_FR);           // Clear flag for INT2 (by writing a logical one) to clear any pending interrupts
      EIMSK |= (1 << Z_MAX_INT);        // Enable external interrupt INT2
    #endif

    // And then enable the Pin Change Interrupt
    // Y_MIN_PIN uses Pin Change Interrupt
    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
      Y_MIN_PCMSK |= (1 << Y_MIN_PCI);  // Set PCINT10 to trigger an interrupt on state change
      PCIFR = (1 << Y_MIN_FR);          // Clear flag (by writing a logical one) to clear any pending interrupts
      PCICR |= (1 << Y_MIN_INT);        // Set PCIE1 to enable PCMSK1 scan, i.e. enable interrupts for PCINT15:8    
    #endif    
  }

  /**
  * Disable interrupts for the limit switches
  */ 
  inline void rampsLimitsDisableInterrups() {

      #if defined(X_MIN_PIN) && X_MIN_PIN > -1    
        EIMSK &= ~(1 << X_MIN_INT);       // Disable external interrupt INT5 to ensure no interrupts are generated
        EIFR = (1 << X_MIN_FR);           // Clear flag for INT5 (by writing a logical one) to clear any pending interrupts        
      #endif

      #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1    
        EIMSK &= ~(1 << Z_MAX_INT);       // Disable external interrupt INT2 to ensure no interrupts are generated
        EIFR = (1 << Z_MAX_FR);           // Clear flag for INT2 (by writing a logical one) to clear any pending interrupts
      #endif

      #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
        Y_MIN_PCMSK &= ~(1 << Y_MIN_PCI); // Disable PCINT10 Interrupt   
        PCIFR = (1 << Y_MIN_FR);          // Clear flag (by writing a logical one) to clear any pending interrupts
      #endif     
  }

  /**
  * Enable the limit switches
  * @param value whether hard limits should be enabled
  */ 
  inline void rampsInitLimits(bool hardLimitEnable) {
    
    #if defined(X_MIN_PIN) && X_MIN_PIN > -1
      SET_INPUT(X_MIN_PIN);    
      #ifdef DISABLE_LIMIT_PIN_PULL_UP
        WRITE(X_MIN_PIN, LOW);  // Normal low operation. Requires external pull-down.
      #else
        WRITE(X_MIN_PIN, HIGH); // Enable internal pull-up resistors. Normal high operation.
      #endif
    #endif
    #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
      SET_INPUT(Y_MIN_PIN);
      #ifdef DISABLE_LIMIT_PIN_PULL_UP
        WRITE(Y_MIN_PIN, LOW);  // Normal low operation. Requires external pull-down.
      #else
        WRITE(Y_MIN_PIN, HIGH); // Enable internal pull-up resistors. Normal high operation.
      #endif
    #endif
    #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
      SET_INPUT(Z_MAX_PIN);
      #ifdef DISABLE_LIMIT_PIN_PULL_UP
        WRITE(Z_MAX_PIN, LOW);  // Normal low operation. Requires external pull-down.
      #else
        WRITE(Z_MAX_PIN, HIGH); // Enable internal pull-up resistors. Normal high operation.
      #endif
    #endif

    if (hardLimitEnable) {
      // enable interrupts for the limit switches
      rampsLimitsEnableInterrups();
    } else {
      // disable interrupts for the limit switches
      rampsLimitsDisableInterrups();
    }  
  }

  /**
  * Return the limit status following the LIMIT_MASK.
  * e.g. X_AXIS is (1<<2) or bit 2, Y_AXIS is (1<<1) or bit 1 and Z_AXIS is (1<<0) or bit 0
  * @return the status on the limit switches as a uint8_t
  */ 
  inline uint8_t rampsCheckLimits() {
    // TODO: wrap witin a "#if defined(___MIN_PIN) && ___MIN_PIN > -1)" clause
    return ((READ(X_MIN_PIN)<<X_LIMIT_BIT)|(READ(Y_MIN_PIN)<<Y_LIMIT_BIT)|(READ(Z_MAX_PIN)<<Z_LIMIT_BIT));
  }

  /**
  * Print the limit status to the serial port
  * @param the status on the limit switches as a uint8_t
  * @param a string to print
  */ 
  inline void rampsPrintLimitStatus(const char *s, uint8_t limit_state) {
      printString(s);
      print_unsigned_int8(limit_state,2,N_AXIS);
  }

#ifdef	__cplusplus
}
#endif

#endif	/* RAMPS_H */

