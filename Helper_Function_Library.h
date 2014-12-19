/**
  * Helper Function Library
  *
  * This library is included for you to write any helper functions
  * that your program will be using. Remember that you will declare 
  * your function names in this .h file, and write the code that
  * will be associated with them in the corresponding
  * "Helper_Function_Library.cpp" file
  *
  */

#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "ME2510_Arduino_Shield_Ports.h"
#include "ME2510_Library.h"



//============================
//         Constants
//============================




//====================================================================================
//                                  HELPER FUNCTIONS
//====================================================================================

/*
 * Include your helper function calls and a short description of what they
 * do in this section.
 *
 */
boolean debounce(boolean lastButton);
//puts in delay for button press



