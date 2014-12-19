#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
  #include "ME2510_Library.h"
#include <stdarg.h>

//=============================
//       Motor Functions
//=============================


boolean set_motor_power(int motor, int power)
{

  // Variables to store the control pins for the specified motor.
  int dir1, dir2, pwm_pin;
  // Variable to store the power after compensating for the battery charge.
  int compensated_power;

  // Assign the appropriate pins to apply the setting.
  if(motor == 0)
  {
    dir1 = MOTOR_0_A_PORT;
    dir2 = MOTOR_0_B_PORT;
    pwm_pin = MOTOR_0_EN_PORT;
  }
  else if(motor == 1)
  {
    dir1 = MOTOR_1_A_PORT;
    dir2 = MOTOR_1_B_PORT;
    pwm_pin = MOTOR_1_EN_PORT;
  } 
  else
  {
    // An invalid motor number was specified, so don't do anything.
    return false;
  }

  // Fix the power setting if it's out of range and also check for
  // an explicit 0 power command. If that's the case, we can just
  // disable the motor and be done.
  if(power == 0)
  {
    digitalWrite(pwm_pin, LOW);
    return true;
  }
  else if(power < -100)
  {
    power = -100;
  }
  else if(power > 100)
  {
    power = 100;
  }

  // Now scale to the range that the analogWrite command uses.
  // (Must multiply first to avoid rounding down to zero!)
  // Note that the multiply doesn't quite overflow an int.
  power = (power * MAX_PWM_VALUE) / 100;

  // Get the compensated power level, being sure to pass a positive value.
  compensated_power = compensate_for_battery_charge(abs(power));
  
  // Based on the sign of the original power, set up the direction
  // control pins.
  if(power < 0)
  {
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
  } 
  else
  {
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
  }
  //Apply the compensated power.
  analogWrite(pwm_pin, compensated_power);

  // Mission accomplished!
  return true;

} // set_motor_power()


boolean brake_motor(int motor)
{

  if(motor == 0)
  {
    digitalWrite(MOTOR_0_A_PORT, HIGH);
    digitalWrite(MOTOR_0_B_PORT, HIGH);
    digitalWrite(MOTOR_0_EN_PORT, HIGH);
  }
  else if (motor == 1)
  {
    digitalWrite(MOTOR_1_A_PORT, HIGH);
    digitalWrite(MOTOR_1_B_PORT, HIGH);
    digitalWrite(MOTOR_1_EN_PORT, HIGH);
  }
  else
  {
    return false;
  }

  return true;

} // brake_motor()


//=============================
//      Solenoid Functions
//=============================

boolean activate_solenoid(int solenoid, int ms, int power /* = 100 */)
{
  // Variable to store the port for the specified solenoid.
  int port;

  // Check for a valid solenoid number.
  if(solenoid == 0)
  {
    port = LOAD_0_PORT;
  }
  else if (solenoid == 1)
  {
    port = LOAD_1_PORT;
  }
  else
  {
    return false;
  }

  // Check that the ms parameter is valid.
  if(ms > 2000)
  {
    ms = 2000;
  }
  else if (ms < 0)
  {
    return false;
  }
  
  // Fix the power setting if it's out of range and also check for
  // an explicit 0 power command. If that's the case, we can just
  // delay for the specified time and be done.
  if(power <= 0)
  {
    delay(ms);
    return true;
  }
  else if(power > 100)
  {
    power = 100;
  }
  
  // Now scale to the range that the analogWrite command uses.
  // (Must multiply first to avoid rounding down to zero!)
  // Note that the multiply doesn't quite overflow an int.
  power = (power * MAX_PWM_VALUE) / 100;

  // Compensate for the battery voltage and activate the solenoid.
  power = compensate_for_battery_charge(power);

  analogWrite(port, power);
  delay(ms);
  digitalWrite(port, LOW);

  return true;

} // activate_solenoid()



//=============================
//     Sensor Functions
//=============================


int read_photo(int photo_number)
{
  // Variable to store the raw reading from the analog port.
  int raw;
  // Variable to store the port number where the sensor is connected.
  int port;

  // First, verify the input and assign the port.
  if (photo_number == 0)
  {
    port = PHOTO_0_PORT;
  }
  else if (photo_number == 1)
  {
    port = PHOTO_1_PORT;
  }
  else
  {
    return INVALID;
  }

  // Get the raw reading and scale to the desired range.
  raw = analogRead(port);
  return (int)( 100L*raw/MAX_ADC_VALUE );

} // read_photo()


boolean wait_for_photo_to_exceed(int photo_number, int threshold)
{
  // Variable to hold the current phototransistor reading.
  int current_reading;

  // First verify the threshold input.
  if( (threshold < 0) || (threshold > 100) )
  {
    return false;
  }

  // Now use a call to read_photo to verify the photo_number input and get
  // the initial reading.
  current_reading = read_photo(photo_number);

  if ( current_reading == INVALID )
  {
    return false;
  }

  // Now wait for the threshold to be exceeded.
  while ( current_reading < threshold )
  {
    // Do a little delay so we aren't polling unnecessarily fast.
    delay(POLLING_DELAY);
    // Then update the reading.
    current_reading = read_photo(photo_number);
  }

  return true;

} // wait_for_photo_to_exceed()



int switch_state(int switch_ID)
{
  // Variables to store the port to read and the readings.
  int port, current_state, prior_state;
  // Loop variable.
  int i;

  // Verify that a valid switch number/name was provided and store the port.
  if (switch_ID == 0)
  {
    port = EXTERNAL_SWITCH_PORT;
  }
  else if (switch_ID == START_BUTTON)
  {
    port = START_BUTTON_PORT;
  }
  else if (switch_ID == STOP_BUTTON)
  {
    port = STOP_BUTTON_PORT;
  }
  else
  {
    return INVALID;
  }

  // Check the current state and then set the prior state to the opposite value
  // so that we will be sure to enter the debouncing loop.
  current_state = digitalRead(port);
  prior_state = !current_state;

  // Check the switch state repeatedly waiting for it to stabilize in case the
  // contacts are bouncing due to a transistion. Set a limit of 10 checks so we
  // won't end up stuck here forever if for some reason the state refuses to stabilize.
  for (i = 0; (i < 10) && (current_state != prior_state); i++)
  {
    delay(DEBOUNCE_TIME);
    prior_state = current_state;
    current_state = digitalRead(port);
  }
  // Now interpret the reading and return the appropriate result.
  if(current_state == LOW)
  {
    // Then the switch is closed.
    return 0;
  }
  else
  {
    return 1;
  }

} // switch_state()


boolean wait_for_switch(int switch_ID, int desired_state)
{
  // Variable to store the switch's current state.
  int current_state;

  // First verify the desired_state input.
  if( (desired_state != CLOSED) && (desired_state != OPEN) )
  {
    return false;
  }

  // Now use a call to switch_state to verify the switch_ID input
  // and get the initial state of the switch.
  current_state = switch_state(switch_ID);

  if (current_state == INVALID)
  {
    return false;
  }

  // Now wait for the desired state.
  while ( current_state != desired_state )
  {
    // Do a little delay so we aren't polling unnecessarily fast.
    delay(POLLING_DELAY);
    // Then update the reading.
    current_state = switch_state(switch_ID);
  }

  return true;

} // wait_for_switch()


int read_potentiometer() 
{
  //Variable Declarations: Declare and initialize the variables below

  //Type: int, Name: potentiometer_reading, Initial Value: 0
  int potentiometer_reading = 0;

  potentiometer_reading = analogRead(POTENTIOMETER_PORT);
  
  potentiometer_reading = map(potentiometer_reading, 0, 1023, 0, 100);

  return potentiometer_reading;

}


//=================================
//        Helper Functions
//=================================

void initialize_shield()
{
  // Set the PWM frequency for the motors to be 31250 Hz. The L298 can handle up
  // to 40 kHz and that will keep the motor's buzzing out of the audible range.
  TCCR2B=TCCR2B & 0b11111000 | 0x01;

  // Configure the motor ports as outputs and initialize them to make sure
  // that none of the motors jump when the board powers up or reboots.
  pinMode(MOTOR_0_EN_PORT, INPUT);
  digitalWrite(MOTOR_0_EN_PORT, HIGH);
  pinMode(MOTOR_0_A_PORT, INPUT);
  digitalWrite(MOTOR_0_A_PORT, HIGH);  
  pinMode(MOTOR_0_B_PORT, OUTPUT);

  pinMode(MOTOR_1_EN_PORT, OUTPUT);
  digitalWrite(MOTOR_1_EN_PORT, LOW);
  pinMode(MOTOR_1_A_PORT, INPUT);
  pinMode(MOTOR_1_B_PORT, INPUT);
  
  digitalWrite(MOTOR_1_A_PORT, HIGH);
  digitalWrite(MOTOR_1_B_PORT, HIGH);

  // For the switches, configure the ports as inputs and then write a HIGH value
  // to them to turn on the internal pull-up resistors.
  pinMode(EXTERNAL_SWITCH_PORT, INPUT);
  digitalWrite(EXTERNAL_SWITCH_PORT, HIGH);
  pinMode(START_BUTTON_PORT, INPUT);
  digitalWrite(START_BUTTON_PORT, HIGH);
  pinMode(STOP_BUTTON_PORT, INPUT);
  digitalWrite(STOP_BUTTON_PORT, HIGH);

  // For the load driver (solenoid) circuits, configure the ports as outputs
  // and make sure they are all driven low.
  pinMode(LOAD_0_PORT, OUTPUT);
  digitalWrite(LOAD_0_PORT, LOW);
  pinMode(LOAD_1_PORT, INPUT);
  digitalWrite(LOAD_1_PORT, HIGH);

  // No need to do anything with the analogs.
} // initialize_shield()


void serial_printf(char *format_string, ...)
{
  // Create a buffer to hold the formatted string.
  char buffer[PRINT_BUFFER_LENGTH];

  // Now do some C black magic to assemble the formatted string into the
  // buffer.
  va_list args;
  va_start(args, format_string);
  vsnprintf(buffer, PRINT_BUFFER_LENGTH, format_string, args);
  va_end(args);

  // The formatting string has now been assembled, so send it over the serial port.
  Serial.begin(9600);
  Serial.print(buffer);
  // Now turn the serial port back off and put the Tx an Rx pins
  // back in input mode with the pull-up resistors on so that
  // switches that use those ports will work.
//  Serial.end();
  // Wait for just a moment before changing the pin states to avoid having the
  // transistions be interpreted as part of the serial stream.
  delay(10);
  pinMode(TX_PORT, INPUT);
  digitalWrite(TX_PORT, HIGH);
  pinMode(RX_PORT, INPUT);
  digitalWrite(RX_PORT, HIGH);
} // serial_print()


void halt_program()
{
  // Variable to keep track of LED states.
  int LED_state = LOW;

  // Put all of the actuators in a disabled state.
  digitalWrite(LOAD_0_PORT, LOW);
  digitalWrite(LOAD_1_PORT, LOW);
  digitalWrite(MOTOR_0_EN_PORT, LOW);
  digitalWrite(MOTOR_1_EN_PORT, LOW);

  // Now just make the RX and TX LEDs blink back and forth as a signal
  // that we are stuck in this function.
  pinMode(RX_PORT, OUTPUT);
  digitalWrite(RX_PORT, LED_state);
  pinMode(TX_PORT, OUTPUT);
  digitalWrite(RX_PORT, !LED_state);
  while(true)
  {
    delay(200);
    LED_state = !LED_state;
    digitalWrite(RX_PORT, LED_state);
    digitalWrite(TX_PORT, !LED_state);
  }

} // halt_program()


boolean blink_LED(int LED, int Hz, int n_times)
{
  // Variable to keep track of what the next state should be.
  int next_state = HIGH;
  // Variable to record the half-period of the blink rate.
  int half_period;
  // Variable to store the port that controls the LED.
  int port = -1;
  // Loop variable.
  int i;

  // Check for a valid LED.
  if( !((LED == RX) || (LED == TX) || (LED == LOAD_0) || 
    (LED == LOAD_1)) )
  {
    return false;
  }

  // Check for a valid frequency.
  if( Hz > 0 )
  {
    half_period = 500/Hz;
  }
  else
  {
    return false;
  }

  // Check for a valid n_times argument. (We'll consider n_times = 0
  // to be invalid since that wouldn't be useful at all.)
  if( n_times <= 0 )
  {
    return false;
  }

  // Map the LED to the port.
  if (LED == RX)
  {
    port = RX_PORT;
  }
  else if (LED == TX)
  {
    port = TX_PORT;
  }
  else if (LED == LOAD_0)
  {
    port = LOAD_0_PORT;
  }
  else if (LED == LOAD_1)
  {
    port = LOAD_1_PORT;
  } // if LED

  // Note that the LOAD LEDs are already configured as outputs, but the TX and RX
  // LEDs will be configured as inputs with their pull-up resistors high causing
  // the LEDs to already be on. To make sure the LEDs actually blink when asked to, 
  // we need to change these to outputs. Otherwise the high-impedance state won't
  // dissipate the charge on the MOSFET gates and the LEDs will just stay on.
  if( (LED == RX) || (LED == TX) )
  {
    pinMode(port, OUTPUT);
    digitalWrite(port, LOW);
    // Do a quater period delay before getting started.
    delay(half_period/2);
  }

  // Now go ahead and do the blinking.  
  for(i=0; i<n_times; i++)
  {
    digitalWrite(port, next_state);
    delay(half_period);
    next_state = !next_state;
    digitalWrite(port, next_state);
    delay(half_period);
    next_state = !next_state;
  } // for

  // Restore the pinMode for TX or RX if it was changed.
  if( (LED == RX) || (LED == TX) )
  {
    pinMode(port, INPUT);
    digitalWrite(port, HIGH); // Turns the pull-up back on.
  }

  return true;

} // blink_LED()


int compensate_for_battery_charge(int pwm_value)
{
  // First obtain the ADC reading that corresponds to the reference level.
  // Define this as a static variable so it won't have to be recomputed each time
  // the function is called.
  // Note that the battery voltage is monitored throught a voltage divider with
  // a 51 and a 100 kOhm resistor.
  // The result will be stored as a long since it will need to be used as one
  // to avoid an overflow later on.
  static const long REFERENCE_ADC_READING =
    (REFERENCE_BATTERY_MV * 51L * MAX_ADC_VALUE)/((long)ADC_REFERENCE_MV*(100+51));

  // We also need a variable to hold the current battery voltage reading.
  int current_battery_adc_reading = analogRead(BATTERY_V_PORT);

  // Also define variables to hold an intermediate result and the final value.
  long temp;
  int compensated_pwm;

  // Check that the current battery reading is at least half the reference
  // level. Otherwise, it must be an unexpected type of battery or the
  // battery isn't plugged in at all. If that occurs, set the current reading
  // to the reference value so the pwm value doesn't get scaled at all.
  // That will avoid having strange things happen due to the unexpected power
  // source.
  if(current_battery_adc_reading < REFERENCE_ADC_READING/2)
  {
    current_battery_adc_reading = REFERENCE_ADC_READING;
  }

  // All we need to do now is scale the input value, but the multiplication 
  // will overflow an int, so use a long for the intermediate result.
  temp = REFERENCE_ADC_READING * pwm_value;
  // Now do the division and convert the result back to an int.
  compensated_pwm = temp/current_battery_adc_reading;

  // Check and make sure that the maximum pwm value has not been exceeded.
  // This may happen if the battery voltage is below the reference level
  // since that results in the command being scaled up.
  if(compensated_pwm > MAX_PWM_VALUE)
  {
    compensated_pwm = MAX_PWM_VALUE;
  }

  // Debugging code:
  // serial_printf("ref_adc = %ld, bat_adc = %d, temp = %ld, pwm = %d, comp_pwm = %d\n", REFERENCE_ADC_READING, current_battery_adc_reading, temp, pwm_value, compensated_pwm);

  return compensated_pwm;

} // compensate_for_battery_charge()
