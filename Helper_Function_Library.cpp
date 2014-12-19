#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "Helper_Function_Library.h"
#include <stdarg.h>

/*
 * In this file, you will write the code for your helper functions.
 * For any functions included in this .cpp file, you must also include the
 * function call in the associated Helper_Function_Library.h file
 *
 */

//====================================================================================
//                                  HELPER FUNCTIONS
//====================================================================================


boolean debounce(boolean lastButton)
{
  boolean current = digitalRead(STOP_BUTTON);
  if(lastButton != current)
  {
    delay(5);
    current = digitalRead(STOP_BUTTON);
  }
  return current;
}
