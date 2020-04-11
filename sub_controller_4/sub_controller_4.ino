/* This is used as the float to send signals down to the submaine */

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


/* Servo options */
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION 8
#define SERVO_MIN 7
#define SERVO_MAX 32
#define FORWARD_DIR 20


/* Rudder */
#define RUDDER_SERVO_PIN 21
#define RUDDER_PWM_CHANNEL 0
int rudderServoPos;


/* Motor */
#define MOTOR_FORWARD_PIN 26
#define MOTOR_BACKWARD_PIN 27
#define MOTOR_FORWARD_PWM_CHANNEL 1
#define MOTOR_BACKWARD_PWM_CHANNEL 2
#define MOTOR_SPEED 120
int motor_state;


/* Stepper motor */
#define STEPPER_STEP_PIN 18
#define STEPPER_DIR_PIN 19
#define STEP_SPEED 5


/* Lights */
#define LIGHTS_PIN 23
int lightsToggle;



void setup() {
  // setup bluetooth or debugging terminal
  Serial.begin(115200);
  SerialBT.begin("submarine_controller"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // Rudder setup
  ledcSetup(RUDDER_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(RUDDER_SERVO_PIN, RUDDER_PWM_CHANNEL);
  set_servo_percentage(&rudderServoPos, 90);
  ledcWrite(RUDDER_PWM_CHANNEL, rudderServoPos);

  // Motor setup
  ledcSetup(MOTOR_FORWARD_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_FORWARD_PIN, MOTOR_FORWARD_PWM_CHANNEL);
  ledcSetup(MOTOR_BACKWARD_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(MOTOR_BACKWARD_PIN, MOTOR_BACKWARD_PWM_CHANNEL);
  ledcWrite(MOTOR_FORWARD_PWM_CHANNEL, 0);
  ledcWrite(MOTOR_BACKWARD_PWM_CHANNEL, 0);

  // Lights setup
  pinMode(LIGHTS_PIN, OUTPUT);
  lightsToggle = 0;
  digitalWrite(LIGHTS_PIN, lightsToggle);

  // Stepper setup
  pinMode(STEPPER_STEP_PIN,OUTPUT); 
  pinMode(STEPPER_DIR_PIN,OUTPUT);

  // Redundant call as everything is set to still above
  sub_stand_still();
}

void loop() {
  //read the bluetooth command if there is one
  if (!recv_cmd()) {
    interpret_bt_state();
    Serial.println(driveState);
    set_state();
  }
  if (driveState == UP || driveState == DOWN) { for (int i = 0; i < 20; i++) {digitalWrite(STEPPER_STEP_PIN,HIGH); delay(STEP_SPEED); digitalWrite(STEPPER_STEP_PIN,LOW); delay(STEP_SPEED);} }
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
  int cmd = (int)(in[0]-'0');
  driveState = cmd;
}


/*
 * Where 90 deg is the middle.
 */
void set_servo_percentage(int* servo, int deg) {
  int set;
  float set_f = (float)( SERVO_MAX - SERVO_MIN ) / 180;
  set = set_f * (float)deg + SERVO_MIN;
  *servo = set;
}


void set_state() {
  // Set the state of the sub
  // If its toggling the lights then dont reset the movement state
  if (driveState == TOGGLE_LIGHTS) {
    lightsToggle = ~lightsToggle;
    digitalWrite(LIGHTS_PIN, lightsToggle);
  } else {
    // First set to nothing to reset all states
    sub_stand_still();
    if (driveState == STAND_STILL) {}
    else if (driveState == FORWARD) { motor_state = 1; }
    else if (driveState == BACKWARD) { motor_state = -1; }
    else if (driveState == LEFT) { 
      set_servo_percentage(&rudderServoPos, 120);
    } else if (driveState == RIGHT) {
      set_servo_percentage(&rudderServoPos, 60);
    } else if (driveState == UP) {
      digitalWrite(STEPPER_DIR_PIN, HIGH);
    } else if (driveState == DOWN) {
      digitalWrite(STEPPER_DIR_PIN, LOW);
    }
    ledcWrite(RUDDER_PWM_CHANNEL, rudderServoPos);
    set_motor();
  }
}


void set_motor() {
  // if this is called, first reset movement to stand still
  ledcWrite(MOTOR_FORWARD_PWM_CHANNEL, 0); ledcWrite(MOTOR_BACKWARD_PWM_CHANNEL, 0);
  if (motor_state == 1) { ledcWrite(MOTOR_FORWARD_PWM_CHANNEL, MOTOR_SPEED); }
  else if (motor_state == -1) { ledcWrite(MOTOR_BACKWARD_PWM_CHANNEL, MOTOR_SPEED); }
}



void sub_stand_still() {
  set_servo_percentage(&rudderServoPos, 90);
  ledcWrite(RUDDER_PWM_CHANNEL, rudderServoPos);
  motor_state = 0;
  set_motor();
}
