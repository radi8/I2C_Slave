/**
  remoteArduino Version 1.0.0 (c) 2021 Graeme Jury ZL2APV

  This Arduino operates as an I2C slave to an ESP01 Master which in turn is a server for a
  remote client running on a PC.Yhe remote client sends commands via the ESP01 via TCP/IP
  wireless to switch relays which enable power to be applied to radio equipment connected
  to the unit. The Arduino sends back an acknowledgement by reading toe relay port value.

  Connections: Arduino analog pins 4 (sda) and 5 (scl) are connected to the ESP01 I2C from the
  gpio0 (sda) and gpio1 (scl) via a 3 volt to 5 volt level translator. A range of inputs/outputs
  are provided with some being via optocouplers and others via switching transistors.

*/

#include <Wire.h>

#define _DEBUG // Comment this out on final compile

#define I2CAddressESPWifi 9

/***************** Global Variables *****************/
boolean last_state = HIGH;

char I2C_sendBuf[32];
char I2C_recBuf[32];
long CMD = 0;       // Commands received are placed in this variable

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
  CMD_ANT_1,      // No antenna selected (Dummy Load)
  CMD_ANT_2,      // Wire
  CMD_ANT_3,      // Mag Loop
  CMD_ANT_4,      // LoG
  CMD_READ_A0,    // Shack voltage
  CMD_READ_A1,
  CMD_READ_A2,
  CMD_READ_D11,    // Digital input via opto-coupler
  CMD_READ_D12,
  CMD_SET_LED_HI,
  CMD_SET_LED_LO,
  CMD_STATUS,
  CMD_ID // Always keep this last
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
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode (A0, INPUT);
  digitalWrite (A0, LOW);  // disable pull-up
  pinMode (A1, INPUT);
  digitalWrite (A1, LOW);  // disable pull-up
  pinMode (A2, INPUT);
  digitalWrite (A2, HIGH);  // enable pull-up

  // Set the output pins. These default to LOW so set HIGH any that
  // need to be LOW active

  pinMode (2, OUTPUT);        // Opto-U7
  pinMode (3, OUTPUT);        // Opto-U6
  pinMode (4, OUTPUT);        // J13 pin4
  pinMode (5, OUTPUT);        // J13 pin3
  pinMode (6, OUTPUT);        // J13 pin2
  pinMode (7, OUTPUT);        // J13 pin1
  pinMode (8, OUTPUT);        // J11 pin3
  pinMode (9, OUTPUT);        // J11 pin2
  digitalWrite (9, HIGH);
  pinMode (10, OUTPUT);       // J11 pin1
  digitalWrite (10, HIGH);
  pinMode (11, INPUT);        // Opto-U5
  pinMode (12, INPUT);        // Opto-U4
  digitalWrite (11, HIGH);    // Enable pullup
  digitalWrite (12, HIGH);    // Enable pullup

  Serial.begin(115200);
  Serial.println("Started slave at address 9");

  Wire.begin(I2CAddressESPWifi); // Set up as slave
  Wire.onReceive(receiveEvent);// Registers a function to be called when a slave
  // device receives a transmission from a master.
  Wire.onRequest(requestEvent);// Register a function to be called when a
  // master requests data from this slave device.
}

void loop()
// There is nothing in the loop as all data received is via an interrupt function.
{
  delay(1);
}

/************************** I2C subroutines **************************/

// The slave is listening for commands from the master. Some commands are
// for the slave to perform a task like set a digital output and other
// commands are for information to be sent back to the master.
// The receiveEvent captures the sent command and places it in the 'CMD'
// global variable for subsequent processing. Every command received is
// expected to generate an acknowledgement and this is retrieved by the master
// sending a requestEvent() serviced by the interrupt driven subroutine.

void receiveEvent(int howMany)
// called by I2C interrupt service routine when incoming data arrives.
// The command is sent as a numeric string and is converted to a long.
{
  char * pEnd;

  memset(I2C_recBuf, NULL, strlen(I2C_recBuf)); // Null the I2C Receive Buffer
  for (byte i = 0; i < howMany; i++)
  {
    I2C_recBuf[i] = Wire.read ();
  }  // end of for loop
#ifdef _DEBUG
  Serial.print("@Slave:receiveEvent(), howMany = "); Serial.print(howMany);
#endif
#ifdef _DEBUG
  Serial.print("; & Loop I2C_recBuf = "); Serial.println(I2C_recBuf);
#endif
  CMD = strtol(I2C_recBuf, &pEnd, 10);
#ifdef _DEBUG
  Serial.print("@Slave:receiveEvent(), CMD = "); Serial.println(CMD); Serial.println();
#endif
}

void requestEvent()
// This is called if the master has asked the slave for information. The
// command to identify which info has been received by receiveEvent and
// placed into the global "CMD" variable.
{
  Serial.print("@Slave::requestEvent(), CMD = "); Serial.print(CMD, 10);
  switch (CMD)
  {
    case CMD_PWR_ON:
      sendSensor(55, CMD_PWR_ON);
      break;
    case CMD_PWR_OFF:
      sendSensor(55, CMD_PWR_OFF);
      break;
    case CMD_RLY1_ON:
      digitalWrite(10, LOW);  // J11 pin 1; CMD = 3
      sendSensor(10, CMD_RLY1_ON);
      break;
    case CMD_RLY1_OFF:
      digitalWrite(10, HIGH); // CMD = 4
      sendSensor(10, CMD_RLY1_OFF);
      break;
    case CMD_RLY2_ON:
      digitalWrite(9, LOW);  // J11 pin 2; CMD = 5
      sendSensor(9, CMD_RLY2_ON);
      break;
    case CMD_RLY2_OFF:
      digitalWrite(9, HIGH); // CMD = 6
      sendSensor(9, CMD_RLY2_OFF);
      break;
    case CMD_RLY3_ON:
      digitalWrite(8, HIGH);  // J11 pin 3; CMD = 7
      sendSensor(8, CMD_RLY3_ON);
      break;
    case CMD_RLY3_OFF:
      digitalWrite(8, LOW); // CMD = 8
      sendSensor(8, CMD_RLY3_OFF);
      break;
    case CMD_RLY4_ON:
      digitalWrite(3, HIGH);  // Opto-Coupler
      sendSensor(3, CMD_RLY4_ON);
      break;
    case CMD_RLY4_OFF:
      digitalWrite(3, LOW);
      sendSensor(3, CMD_RLY4_OFF);
      break;
    case CMD_TUNE_DN:
      // Send a button press to autotuner
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(2, HIGH);
      sendSensor(2, CMD_TUNE_DN);
      break;
    case CMD_TUNE_UP:
      // Send a button release to autotuner
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(2, LOW);
      sendSensor(2, CMD_TUNE_UP);
      break;
    case CMD_ANT_1: // No antenna selected
      digitalWrite(4, LOW); // J13 - 4
      digitalWrite(5, LOW); // J13 - 3
      digitalWrite(6, LOW); // J13 - 2
      digitalWrite(7, LOW); // J13 - 1
      sendSensor(4, CMD_ANT_1);
      break;
    case CMD_ANT_2:
      digitalWrite(4, LOW);
      digitalWrite(5, HIGH); // RLC
      digitalWrite(6, LOW);
      digitalWrite(7, LOW);
      sendSensor(5, CMD_ANT_2);
      break;
    case CMD_ANT_3:
      digitalWrite(4, LOW);
      digitalWrite(5, LOW);
      digitalWrite(7, LOW);
      digitalWrite(6, HIGH); // RLA
      sendSensor(6, CMD_ANT_3);
      break;
    case CMD_ANT_4:
      digitalWrite(4, LOW);
      digitalWrite(6, LOW);
      digitalWrite(7, HIGH); // RLB
      digitalWrite(5, HIGH); // RLA
      sendSensor(7, CMD_ANT_4); // Only use 1 of the 2 selected relays for response
      break;
    case CMD_READ_A0: sendSensor(A0, CMD_READ_A0); break;  // send A0 value
    case CMD_READ_A1: sendSensor(A1, CMD_READ_A1); break;  // send A1 value
    case CMD_READ_A2: sendSensor(A2, CMD_READ_A2); break;  // send A2 value
    case CMD_READ_D11: sendSensor(11, CMD_READ_D11); break;   // get D11 value
    case CMD_READ_D12: sendSensor(12, CMD_READ_D12); break;   // get D12 value
    case CMD_SET_LED_HI:
      digitalWrite(LED_BUILTIN, HIGH);  // J11 pin 2; CMD = 22
      sendSensor(LED_BUILTIN, CMD_SET_LED_HI);
      break;
    case CMD_SET_LED_LO:
      digitalWrite(LED_BUILTIN, LOW);  // J11 pin 2; CMD = 22
      sendSensor(LED_BUILTIN, CMD_SET_LED_LO);
      break;
    case CMD_STATUS:
      sendSensor(55, CMD_STATUS);
      break;
    case CMD_ID:
      sendSensor(55, CMD_ID);
      break;
  }  // end of switch
  
#ifdef _DEBUG
  Serial.print(" & I2C_sendBuf = ");
  Serial.println(I2C_sendBuf);
  Serial.println("-------------------------");
#endif
  CMD = 0;
}

void sendSensor (int myPin, uint8_t cmd)
// 'int myPin' refers to the port affected by the command. The integer value of
// the port is read and converted to a string to be sent to the remote client.
{
  int cmdVal = 0;
  uint8_t len;

  if (myPin == 55) {
    cmdVal = 9;
  } else {
    if (myPin >= A0 && myPin < A4) {
      cmdVal = analogRead (myPin);
    } else {
      cmdVal = digitalRead (myPin);
    }
  }
  memset(I2C_sendBuf, '\0', 32); // Clear the I2C Send Buffer
  sprintf(I2C_sendBuf, "%d %d", cmd, cmdVal);
  len = strlen(I2C_sendBuf);

  I2C_sendBuf[len] = '\0';
  for (byte i = 0; i <= len; i++)
  {
    Wire.write(I2C_sendBuf[i]); // Chug out one char at a time.
  }  // end of for loop
}  // end of sendSensor
