# remoteArduino

This Arduino operates as an I2C slave to an ESP01 Master which in turn is a server for a
remote client running on a PC.Yhe remote client sends commands via the ESP01 via TCP/IP
wireless to switch relays which enable power to be applied to radio equipment connected
to the unit. The Arduino sends back an acknowledgement by reading toe relay port value.
Connections: Arduino analog pins 4 (sda) and 5 (scl) are connected to the ESP01 I2C from the
gpio0 (sda) and gpio1 (scl) via a 3 volt to 5 volt level translator. A range of inputs/outputs
are provided with some being via optocouplers and others via switching transistors.

