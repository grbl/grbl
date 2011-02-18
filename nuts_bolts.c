#include "nuts_bolts.h"
#include <stdint.h>
#include <stdlib.h>

int read_double(char *line, uint8_t *char_counter, double *double_ptr)                  
{
  char *start = line + *char_counter;
  char *end;
  
  *double_ptr = strtod(start, &end);
  if(end == start) { 
    return(false); 
  };

  *char_counter = end - line;
  return(true);
}

