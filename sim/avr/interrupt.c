/*
  interrupt.c - replacement for the avr library of the same name to provide
  dummy register variables

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "interrupt.h"
#include "io.h"

//pseudo-Interrupt vector table  
isr_fp compa_vect[6]={0};
isr_fp compb_vect[6]={0};
isr_fp ovf_vect[6]={0};


void sei() {io.sreg|=SEI;}
void cli() {io.sreg&=~SEI;}



int16_t sim_scaling[8]={0,1,8,64,256,1024,1,1}; //clock scalars
//Timer/Counter modes:  these are incomplete, but enough for this application
enum sim_wgm_mode {
  wgm_NORMAL,
  wgm_CTC,
  wgm_FAST_PWM,
  wgm_PHASE_PWM,
  wgm_PH_F_PWM,
  wgm_RESERVED
};

enum sim_wgm_mode sim_wgm0[4] = {wgm_NORMAL,wgm_PHASE_PWM,wgm_CTC,wgm_FAST_PWM};
enum sim_wgm_mode sim_wgmN[8] = {wgm_NORMAL,wgm_PHASE_PWM,wgm_PHASE_PWM,wgm_PH_F_PWM,
											wgm_CTC, wgm_FAST_PWM, wgm_FAST_PWM, wgm_FAST_PWM};

void timer_interrupts() {
  int i;
  uint8_t ien = io.sreg&SEI;  //interrupts enabled?
  io.prescaler++;
 
  //all clocks
  for (i=0;i<2;i++){

	 uint8_t cs = io.tccrb[i]&7; //clock select bits 
	 int16_t increment = sim_scaling[cs];
	 //check scaling to see if timer fires
	 if (increment && (io.prescaler&(increment-1))==0) { 

		//select waveform generation mode 
		enum sim_wgm_mode mode;
		if (i==0 || i==2) {  //(T0 and T2 are different from rest)
		  uint8_t wgm = io.tccra[i]&3; //look at low 2 bits
		  mode = sim_wgm0[wgm];
		}
		else {
		  uint8_t wgm = ((io.tccrb[i]&8)>>1) | (io.tccra[i]&3); //only using 3 bits for now
		  mode = sim_wgmN[wgm];
		}
		
		//tick
		io.tcnt[i]++;
		//comparators
		if ((io.timsk[i]&(1<<SIM_OCA)) && io.tcnt[i]==io.ocra[i]) io.tifr[i]|=(1<<SIM_OCA);
		if ((io.timsk[i]&(1<<SIM_OCB)) && io.tcnt[i]==io.ocrb[i]) io.tifr[i]|=(1<<SIM_OCB);
		if ((io.timsk[i]&(1<<SIM_OCC)) && io.tcnt[i]==io.ocrc[i]) io.tifr[i]|=(1<<SIM_OCC);
		  

		switch (mode) {
		  case wgm_NORMAL: //Normal mode
			 if (i==0) io.tcnt[i]&=0xFF; //timer0 is 8 bit;
			 if (i==2) io.tcnt[i]&=0xFF; //timer2 is 8 bit;
			 if (io.tcnt[i]==0) io.tifr[i]|=(1<<SIM_TOV); 
			 break;

   	  case wgm_CTC: //CTC mode
			 if (io.tcnt[i]==io.ocra[i]) io.tcnt[i]=0;
			 break;
  		  default:  //unsupported
			 break; 
		}
		//call any triggered interupts
		if (ien && io.tifr[i]) {
		  if (compa_vect[i] && (io.tifr[i]&(1<<SIM_OCA))) {
			 compa_vect[i]();
			 io.tifr[i]&=~(1<<SIM_OCA);
			 //TODO: insert port_monitor call here
		  }
		  if (compb_vect[i] && (io.tifr[i]&(1<<SIM_OCB))) {
			 compb_vect[i]();
			 io.tifr[i]&=~(1<<SIM_OCB);
		  }
		  if (ovf_vect[i] && (io.tifr[i]&(1<<SIM_TOV))) {
			 ovf_vect[i]();
			 io.tifr[i]&=~(1<<SIM_TOV);
		  }
		}
	 }
  }
	 //// TODO for more complete timer sim. 
	 // pwm modes. (only used for variable spindle, I think).
    // -- would require fixing wgm mode for Timers1..5
    // -- phase correct modes need updown counter.
    // output pins (also only for variable spindle, I think).

    //// Other chip features not needed yet for grbl:
	 // writes to TCNT0 prevent compare match (need write detector.)
    // force output compare (unused)
    // input capture (unused and how would we signal it?)
    // define the other output compare registers.
	 // usercode can clear unhandled interrupt flags by writing 1. 
    //  --(this may be impossible, since bit was 1 before the write.)
    // prescaler reset.
    // maybe need to cli on interrupt entry
  
}




