/**
*
* Sample Multi Master I2C implementation.  Sends a button state over I2C to another
* Arduino, which flashes an LED correspinding to button state.
*
* Connections: Arduino analog pins 4 (sda) and 5 (scl) are connected between the two Arduinos,
* with a 1k pullup resistor connected to each line.  Connect a push button between
* digital pin 10 and ground, and an LED (with a resistor) to digital pin 9.
*
*/

#include <Wire.h>

#define LED 13
#define TUNE_OUTPUT 13
#define BUTTON 10

#define I2CAddressESPWifi 9


boolean last_state = HIGH;

char I2C_sendBuf[32];
char I2C_recBuf[32];
long CMD = 0; //Commands received are placed in this variable

// various commands we can receive
enum {
    CMD_TUNE = 1,
    CMD_READ_A0  = 2,
    CMD_READ_A1  = 3,
    CMD_READ_A2  = 4,
    CMD_READ_D2 = 5,    
    CMD_READ_D3 = 6,
    CMD_SET_D8_HI = 7,
    CMD_SET_D8_LO = 8,
    CMD_SET_D9_HI = 9,
    CMD_SET_D9_LO = 10,
    CMD_SET_LED_HI = 11,
    CMD_SET_LED_LO = 12,
    CMD_STATUS = 13,
    CMD_ID = 55          
    };

struct {
  uint8_t tunerState = 0;
  int A0_val = 0;
  int A1_val = 0;
  int A2_val = 0;
  bool D2_val = false;
  bool D3_val = false;
} readings;

void setup() {
 pinMode(LED, OUTPUT);
 digitalWrite(LED, LOW);
 
 pinMode(BUTTON, INPUT);
 digitalWrite(BUTTON, HIGH); //Set pullup

 pinMode (2, INPUT);
 digitalWrite (2, HIGH);  // enable pull-up
 pinMode (A0, INPUT);
 digitalWrite (A0, LOW);  // disable pull-up
 pinMode (A1, INPUT);
 digitalWrite (A1, LOW);  // disable pull-up
 pinMode (A2, INPUT);
 digitalWrite (A2, HIGH);  // enable pull-up

 
Serial.begin(115200);
Serial.println("Started slave at address 9");
 
 Wire.begin(I2CAddressESPWifi); // Set up as slave
 Wire.onReceive(receiveEvent);
 Wire.onRequest(requestEvent);
}

// We run through the loop checking if a command has been received and processing
// if so, plus polling the inputs and recording their state so it can be sent to
// master on request
void loop() {

  if(CMD) {
    switch(CMD) {
      case CMD_TUNE:
        // Send a button press to autotuner
        Serial.print("Main loop, command received = ");  // debug
        Serial.println(CMD, DEC);        // debug
        digitalWrite(LED, HIGH);
        delay(100);
        digitalWrite(LED, LOW);
      break;
      case CMD_SET_D8_HI:
        digitalWrite(8, HIGH);
      break;
      case CMD_SET_D8_LO:
        digitalWrite(8, LOW);
      break;
      case CMD_SET_D9_HI:
        digitalWrite(9, HIGH);
      break;
      case CMD_SET_D9_LO:
        digitalWrite(9, LOW);
      break;
    }
    CMD = 0;
  }
  // Now poll all the inputs
  readings.A0_val = analogRead(A0);
  readings.A0_val = analogRead(A1);
  readings.A0_val = analogRead(A2);
  readings.D2_val = digitalRead(2);
  readings.D3_val = digitalRead(3);
}

/************************** I2C subroutines **************************/

// The slave is listening for commands from the master. Some commands are
// for the slave to perform a task like set a digital output and these are
// dealt with in the main loop. Other commands are for information to be
// sent back to the master and these are dealt with in requestEvent().
// The receiveEvent captures the sent command in CMD for processing by the
// main loop or the requestEvent() interrupt driven subroutine

void receiveEvent(int howMany)
// called by I2C interrupt service routine when incoming data arrives.
// The command is sent as a numeric string and is converted to a long.
{
  char * pEnd;
    
  for (byte i = 0; i < howMany; i++)
    {
      I2C_recBuf[i] = Wire.read ();
    }  // end of for loop
  CMD = strtol(I2C_recBuf, &pEnd, 10);
  Serial.println(CMD);
}

void requestEvent()
// This is called if the master has asked the slave for information. The
// command to identify which info has been received by receiveEvent and
// placed into CMD variable.
{
  switch (CMD)
  {
    case CMD_READ_A0: sendSensor(A0); break;  // send A0 value
    case CMD_READ_A1: sendSensor(A1); break;  // send A1 value
    case CMD_READ_A2: sendSensor(A2); break;  // send A2 value
    case CMD_READ_D2: Wire.write(digitalRead(2)); break;   // send D2 value
    case CMD_READ_D3: Wire.write(digitalRead(3)); break;   // send D3 value
    case CMD_ID: {
      strcpy(I2C_sendBuf, "Slave address = 9");
      int len = strlen(I2C_sendBuf);
      for (byte i = 0; i <= len; i++) {
        Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
      }  // end of for loop
      break;   // send our ID
    }   
  }  // end of switch
  CMD = 0;
}

void sendSensor (const byte which)
// The integer value of the analog port is converted to a string and sent.
{
  int val = analogRead (which);
  uint8_t len;
  
  sprintf(I2C_sendBuf, "%d", val);
  len = strlen(I2C_sendBuf);
  I2C_sendBuf[len] = '\0';
  for (byte i = 0; i <= len; i++)
  {
    Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
  }  // end of for loop
  Serial.println(I2C_sendBuf); // debug
}  // end of sendSensor

