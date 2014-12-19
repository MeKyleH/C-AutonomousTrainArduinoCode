/**
  * ME2510_Library
  *
  * This library provides port definitions, constants, and functions for use
  * with the ME 2510 Arduino shield board version 2.0 (Spring 2011).
  * 
  * Sketches using this library should have the line:
  * #include "ME2510_Library.h"
  * at the top of the file and should call:
  * initialize_shield();
  * as the first line of their setup function.
  *
  */

#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "ME2510_Arduino_Shield_Ports.h"


//============================
//         Names
//============================

// Names to use to refer to the various components on the board
// or different states or actions for sensors, etc. These are defined
// for convenience and their values are irrelevant. Therefore, the 
// values are defined as unusual negative values that are unlikely to
// occur by chance or have unfortunate side-effects if used as a port
// number by mistake.

// Buttons:
const int START_BUTTON = -1001;
const int STOP_BUTTON = -1011;
// LEDs:
const int RX = -1021;
const int TX = -1031;
const int LOAD_0 = -1041;
const int LOAD_1 = -1051;
// Switch states:
const int OPEN = -7007;
const int CLOSED = -4004;
// Other identifiers:
const int INVALID = -3333;


//============================
//         Constants
//============================

// Maximum value that can be reported by the ADC.
const int MAX_ADC_VALUE = 1023;

// ADC reference voltage, in millivolts.
const int ADC_REFERENCE_MV = 5000; // mV

// Maximum PWM value that can be written using analogWrite().
const int MAX_PWM_VALUE = 255;

// Length of the string buffer used for serial printing.
// If you try and print a single message with more than this number
// of characters, the buffer will overflow and bad things will happen.
const int PRINT_BUFFER_LENGTH = 128;

// Amount of time (in ms) to wait between successive checks of a sensor.
// This essentially slows the sampling rate down and avoids having the
// processor check the sensors faster than is needed for the application.
const int POLLING_DELAY = 10; // ms

// Amount of time (in ms) to wait for switches to debounce.
// Readings done with an oscilloscope showed that the lever-arm switches
// and the tactile switches on the shield board both bounce for about 2-3 ms.
// This was done with brand new switches, and that time will probably increase
// with wear, so use the time period below to be on the safe side.
const int DEBOUNCE_TIME =  10; // ms

// Reference battery voltage level. PWM commands to the motors and solenoids will
// be scaled to make the battery appear to them to always be at this voltage level.
// 
// The value is defined in millivolts.
// 
// An appropriate range for NiCd or NiMH batteries is 1100 to 1200 multiplied by the
// number of cells in the battery pack. For ME 1010 and 2510, the batteries have 6 cells giving
// a range of 6600 to 7200. Setting the value higher will provide more power while
// the battery is fresh, but performance degradations due to a low charge will affect the 
// robot's performance earlier in the discharge cycle. Setting the value lower will
// provide less power, but consistent performance over a longer period of time as
// the battery discharges.
const int REFERENCE_BATTERY_MV = 6900; // mV


//=============================
//       Motor Functions
//=============================

/**
  * set_motor_power
  *
  * Function to control motor power. Once the power for a motor is set
  * (doing so takes essentially no time), program execution will resume and 
  * the motor will continue to run at that power until a different motor command
  * is issued.
  *
  * To stop a motor, either set the speed to zero using this function or use the
  * brake_motor function. The motor will stop faster if the brake_motor function
  * is used.
  *
  * Inputs:
  *   int motor = the motor whose power should be set (0 or 1).
  *   int power = the desired power from -100 to 100 with negative values going 
  *               backwards and positive values going forwards. Values outside this
  *               range will be limited to the min or max as appropriate.
  *
  * Outputs:
  *   boolean = true if the power command was applied or false if it was not due to
  *             an invalid motor number being specified.
  */
boolean set_motor_power(int motor, int power);

/**
  * brake_motor
  *
  * Function to brake the specified motor passively by shorting its leads together.
  * Note that no special action is required to turn the brake off before trying
  * to run it again using the set_motor_power function.
  * 
  * Inputs:
  *   int motor = the motor that should be put in brake mode. (0 or 1).
  *
  * Outputs:
  *   boolean = true if the brake command was applied or false if it was not due to
  *             an invalid motor number being specified.
  */
boolean brake_motor(int motor);


//=============================
//      Solenoid Functions
//=============================

/**
  * activate_solenoid
  *
  * Function to activate a solenoid for the specified amount of time in milliseconds.
  * The power at which to activate the solenoid can optionally be specified. If omitted
  * the default value is 100 (full power).
  * The function uses a call to delay, so program execution will be halted for the
  * time during which it is activated. Also note that the function will only activate
  * a solenoid for a maximum of 2000 ms each time it is called to avoid overheating the
  * solenoid or the control circuitry.
  *
  * Inputs:
  *   int solenoid = the solenoid to be activated (0, or 1).
  *   int ms = the number of milliseconds to leave the solenoid activated (2000 max).
  *   int power = optional argument to specify the power at which the solenoid should
  *               be activated. The valid range is 0 to 100. If omitted, a value of
  *               100 will be used as the default. Values outside the valid range will
 *                be limited to the min or max as appropriate.
  *
  * Outputs:
  *   boolean = true if the command was executed or false if it was not due to an
  *             invalid solenoid number or a negative time value being specified.
  */
boolean activate_solenoid(int solenoid, int ms, int power = 100);


//=============================
//     Sensor Functions
//=============================


/**
  * read_photo
  *
  * Function to read the specified phototransistor and scale the raw value reported
  * by the analog-to-digital converter into the range of 0 to 100. When no IR light
  * is applied, the value will be very near 0. When it is saturated with IR light,
  * it will be very near 100.
  *
  * Inputs:
  *   int photo_number = the number of the phototransistor to be read (0 or 1).
  *
  * Outputs:
  *   int = the scaled reading from the phototransistor. If an invalid phototransistor
  *         number is provided, then the function will report the constant INVALID.
  */
int read_photo(int photo_number);


/**
  * wait_for_photo_to_exceed
  *
  * Function that waits for the specified phototransistor's reading to exceed
  * the specified threshold. The function uses read_photo to obtain the 
  * phototransistor reading, so the threshold should be set accordingly.
  *
  * Inputs:
  *   int photo_number = the number of the phototransistor to be read (0 or 1).
  *   int threshold = the threshold that must be exceeded before the function returns.
  *                   This must be between 0 and 100.
  *
  * Outputs:
  *   boolean = true if the input arguments were valid or false if they were not.
  */
boolean wait_for_photo_to_exceed(int photo_number, int threshold);


/**
 * switch_state
 *
 * Function to report whether the specified switch is open or closed.
 * The function delays program execution for at least DEBOUNCE_TIME milliseconds
 * to make sure the switch state is stable before reporting a result.
 *
 * Inputs:
 *   int switch_ID = the switch number or name to check (0, 1, 2, START_BUTTON, or STOP_BUTTON)
 *
 * Outputs:
 *   int  = CLOSED or OPEN if a valid switch identifier was provided. Otherwise the
 *          function will return the constant INVALID.
 */
int switch_state(int switch_ID);


/**
  * wait_for_switch
  *
  * Function that waits for the specified switch to be in the specified state.
  * The function checks the switch state repeatedly until the state occurs
  * and then returns. If the switch is already in the specified state, it will
  * return immediately.
  *
  * Inputs:
  *   int switch_ID = the switch number or name to check (0, 1, 2, START_BUTTON, or STOP_BUTTON)
  *   int desired_state = OPEN or CLOSED
  *
  * Outputs:
  *   boolean = true if the input arguments were valid or false if they were not.
  */
boolean wait_for_switch(int switch_ID, int desired_state);


/**
  * read_potentiometer
  *
  * Function to read the potentiometer's position and scale it to the range
  * of 0 to 100.
  *
  * (There are no inputs.)
  *
  * Outputs:
  *   int = the scaled potentiometer reading ranging from 0 to 100.
  *
  * Example:
  *   int reading;
  *   reading = read_potentiometer();
  */
int read_potentiometer();


//=================================
//        Helper Functions
//=================================

/**
  * initialize_shield()
  *
  * Function that sets up the ports correctly for the 2510 Arduino shield to operate.
  * This function should be called in the "setup" function of any sketch that uses
  * the shield board.
  *
  * (There are no inputs or outputs.)
  */
void initialize_shield();


/**
  * serial_printf
  *
  * Function to handle printing over the serial port taking into account the
  * interaction between the switches and the serial port on the Arduino.
  * The function uses the default baud rate of 9600.
  *
  * When using the shield board, it is important to use this function
  * rather than the Arduino's default Serial library so that you do not
  * accidentally leave the affected digital ports in the wrong state and
  * disable your touch switches.
  *
  * The first argument is a formatting string just like you use with MATLAB's
  * fprintf command. The most common conversion characters are:
  * 
  * %d or %i   for integers (either one works)
  * %g   for floating point numbers (chooses the shorter of fixed point and exponential notation)
  * \n   for a newline
  * \t   for a tab
  *
  * Inputs:
  *   char *format_string = A string to print over the serial port for display on the
  *                serial monitor. The string must be defined using double quotes
  *                ("your string") rather than single quotes like MATLAB uses.
  *   ... = Additional numeric arguments to be formatted according to the specifiers
  *         in the format_string.
  * 
  * (The function does not return anything)
  *
  * Example:
  *   The command:
  *     serial_printf("int_val = %d , float_val = %g\n", 56, 72.56);
  *   will print the following over the serial port:
  *     int_var = 56 , float_val = 72.56
  *   (followed by a newline).
  */
void serial_printf(char *format_string, ...);


/**
  * halt_program
  *
  * Function that stops the program execution by entering an infinite loop.
  * The function makes sure that all of the actuators attached to the shield
  * board are turned off and then makes the TX and RX LEDs blink as an
  * indicator that the program has been halted.
  *
  * (No inputs or outputs.)
  */
void halt_program();


/**
  * blink_LED
  *
  * Function to make an LED on the shield board blink a specified number of times
  * at a specified rate. Program execution will not resume until the blinking is
  * done. Note that making the LOAD LEDs blink triggers the device that is attached
  * to the load connectors as a side effect. So, if your solenoids are plugged in,
  * they will fire along with the LED as it blinks.
  *
  * Inputs:
  *   int LED = RX, TX, LOAD_0, or LOAD_1 to indicate which LED to blink.
  *   int Hz = the frequency (in Hz) at which the LED should blink. It will blink
  *            at a 50% duty cycle.
  *   int n_times = the number of times the LED should blink.
  *
  * Outputs:
  *   boolean = true if the action was complete successfully or false if an invalid
  *             LED was specified.
  */
boolean blink_LED(int LED, int Hz, int n_times);


/**
  * compensate_for_battery_charge
  *
  * Function to scale a pwm_value to compensate for the battery charge level.
  *
  * The pwm_value is scaled by the reference battery level over the actual
  * battery level so that the controlled device will always operate as if
  * the battery voltage was equal to the reference level.
  *
  * Inputs:
  *   int pwm_value = the pwm_value to be sent to the device before battery
  *                   charge compensation.
  *
  * Outputs:
  *   int = the compensated pwm_value.
  */
int compensate_for_battery_charge(int pwm_value);

