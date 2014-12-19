
//============================= Declare Header Libraries Here =============================//

#include "ME2510_Library.h"
#include "Helper_Function_Library.h"
#include <Servo.h>



//============================= Declare Global Variables Here =============================//

//Declare and initialize any global variables that you will use in your function
  Servo myservo;
  int presses = 0;
  int angle = 0;
  boolean lastButton = LOW;
  boolean currentButton = LOW;
  



//================================== Begin Setup Function =================================//

// The setup() function is called when a sketch starts. Use it to initialize variables, 
// pin modes, start using libraries, etc. The setup function will only run once, after 
// each powerup or reset of the Arduino board.

void setup()
{

  initialize_shield(); //Calling the initialize shield function will set all of your 
                       //port definitions properly for use with the Utah Shield Board Rev 2.0
                       
  Serial.begin(9600);

  myservo.attach(9);
  myservo.write(0);
   pinMode(STOP_BUTTON_PORT, INPUT);
  digitalWrite(STOP_BUTTON_PORT, HIGH);


  
 
  


} //end setup() function


//================================== Begin Loop Function ==================================//

// After creating a setup() function, which initializes and sets the initial values, the 
// loop() function does precisely what its name suggests, and loops consecutively, allowing 
// your program to change and respond. Use it to actively control the Arduino board.

void loop()
{
  //currentButton = debounce(lastButton);
  //if(lastButton == LOW && currentButton == HIGH){
  if(digitalRead (STOP_BUTTON_PORT) == LOW){

      presses++;
       if(presses == 5){  
          presses = 0;
    }
    angle = 45 * presses;
      myservo.write(angle);
       delay (200);
 }
   Serial.print (angle);
   Serial.print ("   ");
   Serial.println(presses);
} //end loop() function





