/*
  relay_control.c - relay control methods
  Part of Grbl

  Copyright (c) 2016 Puskin A. Andrei

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

#include "grbl.h"
void relay_init(){
  RELAY_DDR |= (1 << RELAY1_BIT);
  RELAY_DDR |= (1 << RELAY2_BIT);
  RELAY_DDR |= (1 << RELAY3_BIT);
  RELAY_DDR |= (1 << RELAY4_BIT);
  relay_stop();
}

//some &= ~(1 << bit);        //Set 0 in any case
//some |=  (1 << bit);        //Set 1 in any case
//some ^=  (1 << bit);        //Inverting bit
void relay_stop(){
  RELAY_PORT |= ((1<<RELAY1_BIT)|(1<<RELAY2_BIT)|(1<<RELAY3_BIT)|(1<<RELAY4_BIT));
}

void relay_set_state(uint8_t mode){
  switch(mode){      
    case RELAY_DISABLE:relay_stop();break;
    case RELAY_ENABLE:{
        RELAY_PORT &= ~((1<<RELAY1_BIT)|(1<<RELAY2_BIT)|(1<<RELAY3_BIT)|(1<<RELAY4_BIT));
    };break;

    case RELAY_1_ON:RELAY_PORT &= ~(1<<RELAY1_BIT);break;
    case RELAY_2_ON:RELAY_PORT &= ~(1<<RELAY2_BIT);break;
    case RELAY_3_ON:RELAY_PORT &= ~(1<<RELAY3_BIT);break;
    case RELAY_4_ON:RELAY_PORT &= ~(1<<RELAY4_BIT);break;

    case RELAY_1_OFF:RELAY_PORT |=(1<<RELAY1_BIT);break;
    case RELAY_2_OFF:RELAY_PORT |=(1<<RELAY2_BIT);break;
    case RELAY_3_OFF:RELAY_PORT |=(1<<RELAY3_BIT);break;
    case RELAY_4_OFF:RELAY_PORT |=(1<<RELAY4_BIT);break;
  }
}

void relay_run(uint8_t mode){
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
  relay_set_state(mode);
}
