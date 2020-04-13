
#include <String.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
char in[50];

//the state of motion for the submarine
int driveState = 0;
#define STAND_STILL 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define UP 5
#define DOWN 6
#define TOGGLE_LIGHTS 7

// Comms pins
int dataPin = 22;
int testSend = 0;

void setup() {
  // setup bluetooth or debugging terminal
  Serial.begin(115200);
  SerialBT.begin("submarine_controller"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(dataPin, OUTPUT);
}


void loop() {

  //read the bluetooth command if there is one
  if (!recv_cmd()) {
    //interpret and set the command
    interpret_bt_state();
    Serial.println(driveState);
    my_send(driveState, 50);
  }
  delay(10);
}


int recv_cmd() {
  int i = 0;
  while (SerialBT.available() && i < (50 - 2)) {
    in[i++] = (char)SerialBT.read();
  }
  in[i] = '\0';
  if (i > 0) { return 0; }
  else { return 1; }
}


void interpret_bt_state() {
  char cmd = in[0];
  if (cmd == '0') { driveState = 0; }
  else if (cmd == '1') { driveState = 1; }
  else if (cmd == '2') { driveState = 2; }
  else if (cmd == '3') { driveState = 3; }
  else if (cmd == '4') { driveState = 4; }
  else if (cmd == '5') { driveState = 5; }
  else if (cmd == '6') { driveState = 6; }
  else if (cmd == '7') { driveState = 7; }
  else { driveState = 0; }
}

/*
 * Send 3 bits of data
 */
void my_send(uint8_t data, int speed) {
  //send a low to signify the beginning of the protocol
  digitalWrite(dataPin, LOW);
  delay(speed/2);
  digitalWrite(dataPin, HIGH);
  delay(speed/2);

  for (int i = 0; i < 3; i++) {
    if (data & (1<<i)) {
      digitalWrite(dataPin, LOW);
    } else {
      digitalWrite(dataPin, HIGH);
    }
    delay(speed);
  }

  // Reset to high so the slave knows its dormant again
  digitalWrite(dataPin, HIGH);
}
