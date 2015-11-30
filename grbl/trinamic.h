#ifndef TRINAMIC_H
#define TRINAMIC_H

#include <stdbool.h>

typedef struct {
  unsigned long driver_control_register_value;
  unsigned long chopper_config_register;
  unsigned long cool_step_register_value;
  unsigned long stall_guard2_current_register_value;
  unsigned long driver_configuration_register_value;
  //the driver status result
  unsigned long driver_status_result;
        
            
  //the pins for the stepper driver
  unsigned char cs_pin;
                 
  //status values 
  int microsteps; //the current number of micro steps
 
  // config stuff
  unsigned int resistor;
  unsigned int current;
  int constant_off_time;

  // probably not required
  unsigned int number_of_steps;
  bool cool_step_enabled;
} tos100;

// Array of three structs to keep information about the steppers
extern tos100 stepper_tos_100[3];

extern void TMC26XStepper_init(tos100 *tos100);

void TMC26XStepper_start(tos100 *tos100);
void TMC26XStepper_send262(unsigned long datagram, tos100 *tos100);
void TMC26XStepper_setCurrent(tos100 *tos100);
void TMC26XStepper_setMicrosteps(int number_of_steps, tos100 *tos100);
void TMC26XStepper_setConstantOffTimeChopper(char constant_off_time, 
					     char blank_time, 
					     char fast_decay_time_setting, 
					     char sine_wave_offset, 
					     unsigned char use_current_comparator, 
					     tos100 *tos100);
#endif
