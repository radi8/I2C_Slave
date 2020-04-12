/**

  Sample Multi Master I2C implementation.  Sends a button state over I2C to another
  Arduino, which flashes an LED correspinding to button state.

  Connections: Arduino analog pins 4 (sda) and 5 (scl) are connected between the two Arduinos,
  with a 1k pullup resistor connected to each line.  Connect a push button between
  digital pin 10 and ground, and an LED (with a resistor) to digital pin 9.

*/

#include <Wire.h>

#define _DEBUG
#define LED 13
#define A0 14
#define A1 15
#define A2 16
#define TUNE_OUTPUT 13
#define BUTTON 10

#define I2CAddressESPWifi 9

/***************** Global Variables *****************/
boolean last_state = HIGH;

char I2C_sendBuf[32];
char I2C_recBuf[32];
long CMD = 0;       // Commands received are placed in this variable
long LAST_CMD = 0;  // The last command received is held here

enum { // These commands come from tcp client via ESP01 I2C connection
  CMD_PWR_ON = 1, //Start the enum from 1
  CMD_PWR_OFF,
  CMD_RLY1_ON,    // HiQSDR
  CMD_RLY1_OFF,
  CMD_RLY2_ON,    //HermesLite
  CMD_RLY2_OFF,
  CMD_RLY3_ON,    // Linear
  CMD_RLY3_OFF,
  CMD_RLY4_ON,    // Tuner
  CMD_RLY4_OFF,   
  CMD_TUNE_DN,
  CMD_TUNE_UP,
  CMD_ANT_0,    // No antenna selected
  CMD_ANT_1,
  CMD_ANT_2,
  CMD_ANT_3,
  CMD_ANT_4,
  CMD_READ_A0,    // Shack voltage
  CMD_READ_A1,
  CMD_READ_A2,
  CMD_READ_D2,    // Digital input via opto-coupler
  CMD_READ_D3,
  CMD_SET_LED_HI,
  CMD_SET_LED_LO,
  CMD_STATUS,
  CMD_ID // Always keep this last
};

enum { // Send these to tcp client via ESP01
  _pwrSwitch = CMD_ID + 1,
  _tuneState,
  _volts,
  _amps,
  _analog2,
  _digital2,
  _digital3,
  _rly1,
  _rly2,
  _antenna,
  _message
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

 // pinMode (2, INPUT);
 // digitalWrite (2, HIGH);  // enable pull-up
  pinMode (A0, INPUT);
  digitalWrite (A0, LOW);  // disable pull-up
  pinMode (A1, INPUT);
  digitalWrite (A1, LOW);  // disable pull-up
  pinMode (A2, INPUT);
  digitalWrite (A2, HIGH);  // enable pull-up

  // Set the output pins. These default to LOW so set HIGH any that
  // need to be LOW active

  pinMode (2, OUTPUT);          // Opto-U7
  pinMode (3, OUTPUT);          // Opto-U6
  pinMode (4, OUTPUT);          // J13 pin4
  pinMode (5, OUTPUT);          // J13 pin3
  pinMode (6, OUTPUT);          // J13 pin2
  pinMode (7, OUTPUT);          // J13 pin1
  pinMode (8, OUTPUT);          // J11 pin3
  pinMode (9, OUTPUT);          // J11 pin2
  digitalWrite (9, HIGH);
  pinMode (10, OUTPUT);         // J11 pin1
  digitalWrite (10, HIGH);
  pinMode (11, INPUT);         // Opto-U5
  pinMode (12, INPUT);         // Opto-U4
  digitalWrite (11, HIGH);
  digitalWrite (12, HIGH);

  Serial.begin(115200);
  Serial.println("Started slave at address 9");

  Wire.begin(I2CAddressESPWifi); // Set up as slave
  Wire.onReceive(receiveEvent);// Registers a function to be called when a slave
                               // device receives a transmission from a master.
  Wire.onRequest(requestEvent);// Register a function to be called when a
                               // master requests data from this slave device.
}

// We run through the loop checking if a command has been received and processing
// if so, plus polling the inputs and recording their state so it can be sent to
// master on request
void loop() {

  if (CMD != LAST_CMD) {
    Serial.print("@Main loop; CMD = "); Serial.println(CMD);
    LAST_CMD = CMD;
    switch (CMD) {
      case CMD_TUNE_DN:
        // Send a button press to autotuner
        Serial.print("Main loop, command received = ");  // debug
        Serial.println(CMD, DEC);        // debug
        digitalWrite(LED, HIGH);
        digitalWrite(2, HIGH);
        break;
      case CMD_TUNE_UP:
        // Send a button release to autotuner
        digitalWrite(LED, LOW);
        digitalWrite(2, LOW);
        break;
      case CMD_RLY1_ON:
        digitalWrite(10, LOW);  // J11 pin 1
        break;
      case CMD_RLY1_OFF:
        digitalWrite(10, HIGH);
        break;
      case CMD_RLY2_ON:
        digitalWrite(9, LOW);  // J11 pin 2
        break;
      case CMD_RLY2_OFF:
        digitalWrite(9, HIGH);
        break;
      case CMD_RLY3_ON:
        digitalWrite(8, HIGH);  // J11 pin 3
        break;
      case CMD_RLY3_OFF:
        digitalWrite(8, LOW);
        break;
      case CMD_RLY4_ON:
        digitalWrite(3, HIGH);  // Opto-Coupler
        break;
      case CMD_RLY4_OFF:
        digitalWrite(3, LOW);
        break;
      case CMD_ANT_0: // No antenna selected
      digitalWrite(4, LOW); // J13 - 4
      digitalWrite(5, LOW); // J13 - 3
      digitalWrite(6, LOW); // J13 - 2
      digitalWrite(7, LOW); // J13 - 1
      break;
    case CMD_ANT_1:
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, HIGH);
      break;
    case CMD_ANT_2:
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(7, LOW);
      digitalWrite(6, HIGH);
      break;
    case CMD_ANT_3:
      digitalWrite(4, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);
      digitalWrite(5, HIGH);
      break;
    case CMD_ANT_4:
      digitalWrite(5, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);      
      digitalWrite(4, HIGH);
      break;
    }
//    if (LAST_CMD == CMD) CMD = 0; // Test to see if CMD changed while processing switch case.
  }
  /*
  // Now poll all the inputs
  readings.A0_val = analogRead(A0);
  delay(5);
  readings.A0_val = analogRead(A1);
  delay(5);
  readings.A0_val = analogRead(A2);
  readings.D2_val = digitalRead(2);
  readings.D3_val = digitalRead(3);
  sendSensor(A0, _volts);
  sendSensor(A1, _amps);
  */
//  sendStatus();
  delay(1);
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
  Serial.print("@Slave:receiveEvent(), howMany = ");Serial.println(howMany);
  memset(I2C_recBuf, NULL, strlen(I2C_recBuf)); // Null the I2C Receive Buffer
  for (byte i = 0; i < howMany; i++)
  {
    I2C_recBuf[i] = Wire.read ();
    Serial.print("@Slave:receiveEvent(), Loop I2C_recBuf = "); Serial.println(I2C_recBuf[i]);
  }  // end of for loop

  CMD = strtol(I2C_recBuf, &pEnd, 10);
#ifdef _DEBUG  
  Serial.print("@Slave:receiveEvent(), CMD = "); Serial.println(CMD);
//  Serial.print("@Slave:receiveEvent(), I2C_recBuf = ");
//  Serial.println(I2C_recBuf);
  Serial.println("-------------------------");
#endif
}

void requestEvent()
// This is called if the master has asked the slave for information. The
// command to identify which info has been received by receiveEvent and
// placed into the global "CMD" variable.
{
    Serial.print("@Slave:requestEvent(), CMD = "); Serial.println(CMD, 10);
  switch (CMD)
  {
    case CMD_READ_A0: sendSensor(A0, _volts); break;  // send A0 value
    case CMD_READ_A1: sendSensor(A1, _amps); break;  // send A1 value
    case CMD_READ_A2: sendSensor(A2, _analog2); break;  // send A2 value
    case CMD_READ_D2: sendSensor(2, _digital2); break;   // send D2 value
    case CMD_READ_D3: sendSensor(3, _digital3); break;   // send D3 value
//    case CMD_STATUS: sendStatus();
    case CMD_ID: {
        memset(I2C_sendBuf, NULL, strlen(I2C_sendBuf)); // Clear the I2C Send Buffer
//        Serial.print("I2C_sendBuf[20] at start =  "); // debug
//        Serial.println(I2C_sendBuf[20], 16); // debug
        sprintf(I2C_sendBuf, "%d Slave address = 9", _message);
//        strcpy(I2C_sendBuf, " Slave address = 9");
#ifdef _DEBUG
  Serial.print("@Slave::requestEvent() I2C_sendBuf = ");
  Serial.println(I2C_sendBuf);
#endif        
    /*
        int len = strlen(I2C_sendBuf);
        I2C_sendBuf[len] = '\0';
        for (byte i = 0; i <= len; i++) {
          Wire.write(I2C_sendBuf[i]); // Chug out 1 character at a time
        }  // end of for loop
        /*    Wire.write(I2C_sendBuf);
              Serial.print("@Slave:requestEvent(), Response sent = ");
              Serial.println(I2C_sendBuf); */
          Wire.write(I2C_sendBuf);
        break;   // send our ID
      }
  }  // end of switch
  CMD = 0;
}

void sendSensor (const byte which, uint8_t cmd)
// The integer value of the analog port is converted to a string and sent.
{
  int val = 0;
  uint8_t len;

  if(which < A0) {
    val = digitalRead (which);
  } else {
    val = analogRead (which);
  }
  memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
  sprintf(I2C_sendBuf, "%d %d", cmd, val);
  len = strlen(I2C_sendBuf);
  
  I2C_sendBuf[len] = '\0';
  for (byte i = 0; i <= len; i++)
  {
    Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
  }  // end of for loop


//  Wire.write(I2C_sendBuf);
//  Serial.println(I2C_sendBuf); // debug
}  // end of sendSensor
/*
void sendStatus()
{
  int x;

  memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
  sendSensor(analogRead(A0), _volts); // send A0 value
  delay(5);
  sendSensor(analogRead(A1), _amps); // send A1 value
  delay(5);
  sendSensor(analogRead(A2), _analog2); // send A2 value
  x = digitalRead(2); // send D2 value
  sprintf(I2C_sendBuf, "%d %d", _digital2, x);
  Wire.write(I2C_sendBuf);
  x = digitalRead(3); // send D2 value
  sprintf(I2C_sendBuf, "%d %d", _digital3, x);
  Wire.write(I2C_sendBuf);
}
*/
