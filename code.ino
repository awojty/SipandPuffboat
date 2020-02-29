
// ----------------------------------------------------------------
// SIP & PUFF 

// ----------------------------------------------------------------

// ----------------------------------------------------------------
//sensor - mprls
#include <Wire.h>
#include "Adafruit_MPRLS.h"

// You dont *need* a reset and EOC pin for most uses, so we set to -1 and don't connect
#define RESET_PIN  -1  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN    -1  // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

//  create an SSC sensor with I2C address 0x78 and power pin 8.

//setup stepper motro
#include <Stepper.h>
int max_speed=20;
int currentSpeed=10;
 
int in1Pin = 12;
int in2Pin = 11;
int in3Pin = 10;
int in4Pin = 9;
 
Stepper motor(512, in1Pin, in2Pin, in3Pin, in4Pin);  


// PIN ASSIGNMENTS
// ----------------------------------------------------------------
int PUFFinput = 9;
int SIPinput = 10;


//joystick
//https://create.arduino.cc/projecthub/arduino-applications/stepper-motor-control-with-joystick-f5feb1
int s1 = 5;
int s2 = 6;
int s3 = 7;
int s4 = 8;
int x;

//SWITCHES TO SELECT joystic or sip and puff 
int inPin_switch = 2;         // the number of the input pin
int outPin_switch = 13;       // the number of the output pin

// ----------------------------------------------------------------
// TIME CONSTANTS - For 16MHz Oscillator
// ----------------------------------------------------------------
long VeryLongCt = 100000; //Approximately 2 seconds
long MinLongCt = 20000; //Approximately 0.6 second
long DebounceCt = 4000; //Approximately 0.1 second

// ----------------------------------------------------------------
// ----------------------------------------------------------------
void setup() {
    


/// sensor  
  Serial.begin(115200);
  Wire.begin();
  Serial.println("MPRLS Simple Test");
  if (! mpr.begin()) {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");
  
  float ambientPressure = mpr.readPressure();
  delay(100);

  
  
  
  
  //stepper
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
 
  // this line is for Leonardo's, it delays the serial interface
  // until the terminal window is opened
 
  Serial.begin(9600);


  //  set min / max reading and pressure, see datasheet for the values for your 
  //  sensor

  //  start the sensor
  Serial.print("start()\t\t");
  Serial.println(ssc.start());
    
///joystick and stepper
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(s4, OUTPUT);
  pinMode(A0, INPUT);
  






      
  Serial.begin(115200);                           //Setting baud rate for serial communication which is used for diagnostic data returned from Bluetooth and microcontroller
  Serial1.begin(115200);                          //Setting baud rate for Bluetooth AT command 

  //pinMode(LED_1_PIN, OUTPUT);                     //Set the LED pin 1 as output(GREEN LED)
  //pinMode(LED_2_PIN, OUTPUT);                     //Set the LED pin 2 as output(RED LED)
         //Set the transistor pin as output
  //pinMode(PIO4_PIN, OUTPUT);                      //Set the bluetooth command mode pin as output

                 //Define Force sensor pinsas input ( Down FSR )

  //pinMode(MODE_SELECT_PIN, INPUT_PULLUP);         // LOW: USB (default with jumper in) HIGH: Bluetooth (jumper removed)
  //pinMode(BUTTON_UP_PIN, INPUT_PULLUP);           //Set increase cursor speed button pin as input
  //pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);         //Set decrease cursor speed button pin as input






    /////





pinMode( SIPinput, INPUT );
pinMode( PUFFinput, INPUT );

pinMode(inPin_switch, INPUT);
pinMode(outPin_switch, OUTPUT);

digitalWrite( SIPinput, HIGH ); //Turn on 20k pullup resistors
digitalWrite( PUFFinput, HIGH ); } //to simplify switch input

// ----------------------------------------------------------------
// ----------------------------------------------------------------
void loop() {

//switch
//choose either joystick or sip
  reading = digitalRead(inPin_switch);

  if (reading == HIGH) {
    bool Flag=false;	#if output is high then joystick 
  else{
    bool Flag=true;		#if output is low then sip n puff}
     
}
if (Flag==false){
    //joystick code 
  x = analogRead(A0);
  
  if(x<500)
  {
 digitalWrite(s1, HIGH);
 digitalWrite(s2, LOW);
 digitalWrite(s3, LOW);
 digitalWrite(s4, LOW);
 delay(5);
 digitalWrite(s1, LOW);
 digitalWrite(s2, HIGH);
 digitalWrite(s3, LOW);
 digitalWrite(s4, LOW);
 delay(5);
 digitalWrite(s1, LOW);
 digitalWrite(s2, LOW);
 digitalWrite(s3, HIGH);
 digitalWrite(s4, LOW);
 delay(5);
 digitalWrite(s1, LOW);
 digitalWrite(s2, LOW);
 digitalWrite(s3, LOW);
 digitalWrite(s4, HIGH);
 delay(5);
 }
 else if(x>550)
 {
  digitalWrite(s1, LOW);
 digitalWrite(s2, LOW);
 digitalWrite(s3, LOW);
 digitalWrite(s4, HIGH);
 delay(5);
 digitalWrite(s1, LOW);
 digitalWrite(s2, LOW);
 digitalWrite(s3, HIGH);
 digitalWrite(s4, LOW);
 delay(5);
 digitalWrite(s1, LOW);
 digitalWrite(s2, HIGH);
 digitalWrite(s3, LOW);
 digitalWrite(s4, LOW);
 delay(5);
 digitalWrite(s1, HIGH);
 digitalWrite(s2, LOW);
 digitalWrite(s3, LOW);
 digitalWrite(s4, LOW);
 delay(5);
 }
 else if(x > 500 && x < 550)
 {
 digitalWrite(s1, LOW);
 digitalWrite(s2, LOW);
 digitalWrite(s3, LOW);
 digitalWrite(s4, LOW);
 }
}
  
if(Flag==true)	// sip control
{
  float pressure_hPa = mpr.readPressure();	//read current pressure
  //if pressure not within margin of error, generate motor velocity between -1500 and 1500
  if (pressure_hPa >= ambientPressure+5 || pressure_hPa <= ambientPressure-5)
  	velocity = (pressure_hPa-ambientPressure)%1500
  else
    velocity = 0.0
  motor.setSpeed(velocity) 
}
    
    

if(Flag==true) //sip control
{//pseudo code
  //read the pressure strength, set the speed of the motor proportional to the speed
  float pressure_hPa = mpr.readPressure();
  delay(100);
  //to be measured at the beginning of teh setup loop, dont hardcode 
  int pressureDifference;
  pressureDifference=pressure_hPa-ambientPressure;
  float factor;
  factor=pressureDifference/ambientPressure+1;
  
  Serial.print("Pressure (hPa): "); 
  Serial.println(pressure_hPa);
  Serial.print("Pressure (PSI): ");
  Serial.println(pressure_hPa / 68.947572932);
  delay(1000);
  motor.setSpeed(factor*20);
  bool Flag1=true;//change the speed flag 

  if(pressure_hPa-ambientPressure<-10){

      float new_speed=-(currentSpeed+factor*20);
      motor.setSpeed(new_speed);
      delay(200);
      currentspeed= new_speed;
      current_reading = mpr.readPressure();
  if( pressureDifference>10){
    float new_speed=(currentSpeed+factor*20);
    motor.setSpeed(new_speed);

      }
      

}


}

// ----------------------------------------------------------------
// END OF CODE SEGMENT
// ----------------------------------------------------------------


//https://www.instructables.com/id/Arduino-Controlled-SIP-PUFF-Switch/?fbclid=IwAR3jvFi6FkQGaI4c5guEvFg-p2yS8j0Ycy4vBAsvxtt7ivxsAI9esh70yUk


// MPRLS Lib
