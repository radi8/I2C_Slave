/**

  Sample Multi Master I2C implementation.  Sends a button state over I2C to another
  Arduino, which flashes an LED correspinding to button state.

  Connections: Arduino analog pins 4 (sda) and 5 (scl) are connected between the two Arduinos,
  with a 1k pullup resistor connected to each line.  Connect a push button between
  digital pin 10 and ground, and an LED (with a resistor) to digital pin 9.

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

enum { // Send these to tcp client via ESP01
  _pwrSwitch = 1,
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

enum { // These commands come from tcp client via ESP01
  CMD_PWR_ON = 1,
  CMD_PWR_OFF,
  CMD_TUNE,
  CMD_READ_A0,
  CMD_READ_A1,
  CMD_READ_A2,
  CMD_D2_HI,
  CMD_D2_LO,
  CMD_D3_HI,
  CMD_D3_LO,
  CMD_SET_RLY1_ON,
  CMD_SET_RLY1_OFF,
  CMD_SET_RLY2_ON,
  CMD_SET_RLY2_OFF,
  CMD_SET_LED_HI,
  CMD_SET_LED_LO,
  CMD_STATUS,
  CMD_ID
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

  if (CMD) {
    switch (CMD) {
      case CMD_TUNE:
        // Send a button press to autotuner
        Serial.print("Main loop, command received = ");  // debug
        Serial.println(CMD, DEC);        // debug
        digitalWrite(LED, HIGH);
        delay(500);
        digitalWrite(LED, LOW);
        break;
      case CMD_SET_RLY1_ON:
        digitalWrite(8, HIGH);
        break;
      case CMD_SET_RLY1_OFF:
        digitalWrite(8, LOW);
        break;
      case CMD_SET_RLY2_ON:
        digitalWrite(9, HIGH);
        break;
      case CMD_SET_RLY2_OFF:
        digitalWrite(9, LOW);
        break;
    }
    CMD = 0;
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
  sendStatus();
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

  for (byte i = 0; i < howMany; i++)
  {
    I2C_recBuf[i] = Wire.read ();
  }  // end of for loop
  CMD = strtol(I2C_recBuf, &pEnd, 10);
  Serial.print("@Slave:receiveEvent(), CMD = ");
  Serial.println(CMD);
}

void requestEvent()
// This is called if the master has asked the slave for information. The
// command to identify which info has been received by receiveEvent and
// placed into the global "CMD" variable.
{
  //  Serial.print("@Slave:requestEvent(), CMD = ");
  //  Serial.println(CMD, 10);
  switch (CMD)
  {
    case CMD_READ_A0: sendSensor(A0, _volts); break;  // send A0 value
    case CMD_READ_A1: sendSensor(A1, _amps); break;  // send A1 value
    case CMD_READ_A2: sendSensor(A2, _analog2); break;  // send A2 value
//    case CMD_READ_D2: Wire.write(digitalRead(2)); break;   // send D2 value
//    case CMD_READ_D3: Wire.write(digitalRead(3)); break;   // send D3 value
    case CMD_STATUS: sendStatus();
    case CMD_ID: {
        memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
        strcpy(I2C_sendBuf, "Slave address = 9");
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
  int val = analogRead (which);
//  uint8_t len;

  memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
  sprintf(I2C_sendBuf, "%d %d", cmd, val);
//  len = strlen(I2C_sendBuf);
/*  
  I2C_sendBuf[len] = '\0';
  for (byte i = 0; i <= len; i++)
  {
    Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
  }  // end of for loop

*/
  Wire.write(I2C_sendBuf);
//  Serial.println(I2C_sendBuf); // debug
}  // end of sendSensor

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

