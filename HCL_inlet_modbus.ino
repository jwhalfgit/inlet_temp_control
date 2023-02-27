//Modbus slave temp controller for HCL instrument SJA2022

//Serial port config: baud = 9600, 8N1 no flow control
//Modbus ID set in setup, but normally 1
//modbus registers are:
  //0 = contact closure from ZERO valve
  //1 = manual trigger a ramp
  //2 = Input1 - the temperature read from the pt100
  //3 = Output1 - the pwm value send to the heater
  //4 = Setpoint1 - the degC setpoint of the PID
  //5 = OS - the operational setpoint degC during normal isothermal control
  //6 = P
  //7 = I
  //8 = D
  //9 = R_P   -the P value during the ramp
  //10 = R_I   -the I value " "
  //11 = R_D  -the D value " "
  //12 = R_Setpoint - the degC temp value during the ramp
  //13 = R_duration - how long it should maintain the ramp setpoint for before reverting to 'normal' temp
  //14 = Input2  - the temperature read from the second pt100
  //15 = Input3  - the temperature read from the third pt100


#include <Adafruit_MAX31865.h>
#include <PID_v1.h>   //the PID loop for the heaters
#include <SPI.h>      //SPI for pt100 boards
#include <millisDelay.h> //a nice easy timer library
#include <avr/wdt.h> //watchdog timer

#include <Modbus.h>  //modbus library
#include <ModbusSerial.h> //modbus using the serial port (usb)

// ModbusSerial object - shortens the name to 'mb'
ModbusSerial mb;

#define PIN_OUTPUT1 3 //the output pin we will pwm on 
#define CC_pin 4 //the pin that the contact closure is connected to

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 max1 = Adafruit_MAX31865(10, 11, 12, 13);
Adafruit_MAX31865 max2 = Adafruit_MAX31865(9, 11, 12, 13);
Adafruit_MAX31865 max3 = Adafruit_MAX31865(8, 11, 12, 13);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0
// initial values
#define RAMP_DURATION 60  //this is now in seconds
#define RAMP_SETPOINT 100  //degC
#define OPERATIONAL_SETPOINT 50  //degC 

//Define Variables for PID
//double temp1, temp2, temp3;   //signed floating point from pt100
double Setpoint1, Input1, Output1, P1, I1, D1;
double Input2, Input3;

// pt100 fault
uint8_t fault1;

// ramp value to wait until after cal valve closed
int rampval = 0;

//Define the aggressive and conservative Tuning Parameters
double aggKp=15, aggKi=0, aggKd=0;
//double aggKp=4, aggKi=0.2, aggKd=1;
//double aggKp=0.5, aggKi=0.05, aggKd=0.25;
double consKp=2, consKi=0.1, consKd=0.5;

//Specify the links and initial tuning parameters
PID myPID1(&Input1, &Output1, &Setpoint1, consKp, consKi, consKd, DIRECT);


//setup the timers
millisDelay ramp_timer; 

//debounce variables
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 500;    // the debounce time; increase if the output flickers
int buttonState;            // the current reading from the input pin
int lastButtonState = 0;  // the previous reading from the input pin

//give modbus reg numbers names
  const int CC = 0;     //0 = contact closure from ZERO valve
  const int MT = 1;     //1 = manual trigger a ramp
  const int T1 = 2;     //2 = Input1
  const int out1 = 3;   //3 = Output1
  const int set1 = 4;   //4 = Setpoint1 
  const int OS = 5;     //5 isothermal temp
  const int P = 6;      //6 = P
  const int I = 7;      //7 = I
  const int D = 8;      //8 = D
  const int R_P = 9;    //9 = R_P
  const int R_I = 10;   //10 = R_I
  const int R_D = 11;   //11 = R_D
  const int R_set = 12; //12 = R_Setpoint
  const int R_dur = 13; //13 = R_duration
  const int T2 = 14;    // 2nd pt100
  const int T3 = 15;    // 3rd pt100
  const int timer = 16;  //time remaining on timer

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SETUP
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup() {

    watchdogSetup();
    // Config Modbus Serial (port, speed, byte format) 
    mb.config(&Serial, 9600, SERIAL_8N1);
    // Set the Slave ID (1-247)
    mb.setSlaveId(1);

  //  Serial.begin(38400);
  //Serial.println("serial begin");
 
  max1.begin(MAX31865_2WIRE);
  max2.begin(MAX31865_2WIRE);
  max3.begin(MAX31865_2WIRE);

  // wait for MAX chip to stabilize
 // delay(500);

  // define the contact closure pin as an input - pull it low to trigger
  pinMode(CC_pin,INPUT_PULLUP);

  //add the modbus registers 
  for(int i=0; i<=16; i++)
  {
  mb.addHreg(i);  //adds registers from 0-15
  }

  Setpoint1 = OPERATIONAL_SETPOINT;
  mb.Hreg(set1, Setpoint1);
  mb.Hreg(CC, 0);
  mb.Hreg(MT, 0);
  mb.Hreg(P, consKp*100); 
  mb.Hreg(I, consKi*100);
  mb.Hreg(D, consKd*100);
  mb.Hreg(R_P, aggKp*100); 
  mb.Hreg(R_I, aggKi*100);
  mb.Hreg(R_D, aggKd*100);
  mb.Hreg(R_set, RAMP_SETPOINT);
  mb.Hreg(R_dur, RAMP_DURATION);
  mb.Hreg(OS, OPERATIONAL_SETPOINT);
  
  
  //turn the PID loops on
  myPID1.SetMode(AUTOMATIC);

  // temperature_ramp(100, 60000);
  //Serial.println("setup");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() {

  wdt_reset();  //need to initialise this?
  
  mb.task(); //sync the modbus registers

  update_values();  //change to update from modbus register sync

//contact closure is pulled high, take it low to GND to trigger it
if( ((mb.Hreg(CC) == 1) || (mb.Hreg(MT) == 1)) && (!ramp_timer.isRunning()) ) {
    rampval = 1;
  }
  if(((mb.Hreg(CC) == 0)&&(rampval == 1))&&(!ramp_timer.isRunning())){
    ramp_timer.start(mb.Hreg(R_dur)*1000);  //start a timer convert seconds to mSec
    
    rampval = 0;
  }  

  if(ramp_timer.isRunning()){
    Setpoint1 = mb.Hreg(R_set);
    myPID1.SetTunings((mb.Hreg(R_P)/100), (mb.Hreg(R_I)/100), (mb.Hreg(R_D)/100));
   ; 
  }
  if(ramp_timer.justFinished()){
    mb.Hreg(MT,0);
    Setpoint1 = mb.Hreg(OS);
    myPID1.SetTunings((mb.Hreg(P)/100), (mb.Hreg(I)/100), (mb.Hreg(D)/100));  
  }

   temperature_control();

 //Serial.println("main loop");
}


////////////////////////////////////////////////////////////////////////////////////////////////////////

void temperature_control(){  

  
  Input1 = max1.temperature(100, RREF);
  // Read additional temperature sensors
  Input2 = max2.temperature(100, RREF);
  Input3 = max3.temperature(100, RREF);

 
  // Check and print any faults on amp 1
  fault1 = max1.readFault();
    max1.clearFault();

  myPID1.Compute();

  //only if no faults set the SSR PWM
  if(fault1 == 0){
    analogWrite(PIN_OUTPUT1, Output1);
  }
  else{
    Output1 = 0;
    analogWrite(PIN_OUTPUT1, 0);
  }
  //pass values to the modbus registers
  mb.Hreg(out1, Output1);
  mb.Hreg(T1, Input1*100);
  mb.Hreg(T2, Input2*100);
  mb.Hreg(T3, Input3*100);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void update_values(){

//check for a contact closure
int trigger = !digitalRead(CC_pin);
// If the switch changed, due to noise or pressing:
if (trigger != lastButtonState) {
  // reset the debouncing timer
  lastDebounceTime = millis();
}

if ((millis() - lastDebounceTime) > debounceDelay) {
  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:
    if (trigger == 1) {
      mb.Hreg(CC,1);
    }
    else{
      mb.Hreg(CC,0);
    }
}
lastButtonState = trigger;  

//update the modbus value of setpoint
mb.Hreg(set1, Setpoint1); 
mb.Hreg(timer, ramp_timer.remaining());

if(!ramp_timer.isRunning())

  myPID1.SetTunings((mb.Hreg(P)/100), (mb.Hreg(I)/100), (mb.Hreg(D)/100));
  
  if(mb.Hreg(OS)!=Setpoint1){
  
    Setpoint1 = mb.Hreg(OS);
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void watchdogSetup(void) 
{  
cli();     // disable all interrupts
wdt_reset(); // reset the WDT timer
/*
 WDTCSR configuration:
 WDIE = 1: Interrupt Enable
 WDE = 1 :Reset Enable
 WDP3 = 0 :For 2000ms Time-out
 WDP2 = 1 :For 2000ms Time-out
 WDP1 = 1 :For 2000ms Time-out
 WDP0 = 1 :For 2000ms Time-out
*/
// Enter Watchdog Configuration mode:
WDTCSR |= (1<<WDCE) | (1<<WDE);
// Set Watchdog settings:
 WDTCSR = (1<<WDIE) | (1<<WDE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
sei();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(WDT_vect) // Watchdog timer interrupt.
{
// Include your code here - be careful not to use functions they may cause the interrupt to hang and
// prevent a reset.
analogWrite(PIN_OUTPUT1, 0);

}
