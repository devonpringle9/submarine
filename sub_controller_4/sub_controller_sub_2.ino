/*
 * Code for on the submarine to receive signals and control the sub motors.
 */


/* Communications */
#define RECV_PIN 32
#define RECV_SPEED 50     // in ms
int nowTime;
int dataBits;


/* Control signals */
#define STAND_STILL 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define UP 5
#define DOWN 6
#define TOGGLE_LIGHTS 7


/* Servos */
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_RESOLUTION 8
#define SERVO_MIN 7
#define SERVO_MAX 32
#define FORWARD_DIR 20

#define LEFT_SERVO_PIN 18
#define RIGHT_SERVO_PIN 19
#define RUDDER_SERVO_PIN 21

#define LEFT_PWM_CHANNEL 0
#define RIGHT_PWM_CHANNEL 1
#define RUDDER_PWM_CHANNEL 2

int leftServoPos;
int rightServoPos;
int rudderServoPos;


/* Motor */
#define MOTOR_FORWARD_PIN 26
#define MOTOR_BACKWARD_PIN 27
#define MOTOR_FORWARD_PWM_CHANNEL 3
#define MOTOR_BACKWARD_PWM_CHANNEL 4
#define MOTOR_SPEED 120
int motor_state;


/* Lights */
#define LIGHTS_PIN 23
int lightsToggle;




void recv_signal() {
  // The pin has been pulled low
  // Start the timer
  nowTime = 0;
  while (!digitalRead(RECV_PIN)) {
    nowTime++;
    delay(1);
  }
  
  // Decide if this was the start bit
  if (nowTime < (RECV_SPEED / 2 + 5)) {
    // Wait for rest of start bits
    delay(RECV_SPEED/2);
    // Now read the next three bits
    delay(RECV_SPEED/2);
    dataBits = 0;
    for (int i = 0; i < 3; i++) {
      Serial.println(!digitalRead(RECV_PIN));
      dataBits = dataBits | (!digitalRead(RECV_PIN) << i);
      delay(RECV_SPEED);
    }
  }
}


void set_motor() {
  // if this is called, first reset movement to stand still
  ledcWrite(MOTOR_FORWARD_PWM_CHANNEL, 0); ledcWrite(MOTOR_BACKWARD_PWM_CHANNEL, 0);
  if (motor_state == 1) { ledcWrite(MOTOR_FORWARD_PWM_CHANNEL, MOTOR_SPEED); }
  else if (motor_state == -1) { ledcWrite(MOTOR_BACKWARD_PWM_CHANNEL, MOTOR_SPEED); }
}


void set_activators() {
  ledcWrite(LEFT_PWM_CHANNEL, leftServoPos);
  ledcWrite(RIGHT_PWM_CHANNEL, rightServoPos);
  ledcWrite(RUDDER_PWM_CHANNEL, rudderServoPos);
  set_motor();
}


void sub_stand_still() {
  set_servo_percentage(&leftServoPos, 90);
  set_servo_percentage(&rightServoPos, 90);
  set_servo_percentage(&rudderServoPos, 90);
  motor_state = 0;
  set_activators();
}


/*
 * Where 90 deg is the middle.
 */
void set_servo_percentage(int* servo, int deg) {
  int set;
  float set_f = (float)( SERVO_MAX - SERVO_MIN ) / 180;
  set = set_f * (float)deg + SERVO_MIN;
  Serial.print("setting servo to ");
  Serial.println(set);
  *servo = set;
}


void set_state() {
  // Set the state of the sub
  // If its toggling the lights then dont reset the movement state
  if (dataBits == TOGGLE_LIGHTS) {
    lightsToggle = ~lightsToggle;
    digitalWrite(LIGHTS_PIN, lightsToggle);
  } else {
    // First set to nothing to reset all states
    sub_stand_still();
    if (dataBits == STAND_STILL) {}
    else if (dataBits == FORWARD) { motor_state = 1; }
    else if (dataBits == BACKWARD) { motor_state = -1; }
    else if (dataBits == LEFT) { 
      set_servo_percentage(&leftServoPos, 110);
      set_servo_percentage(&rightServoPos, 70);
      set_servo_percentage(&rudderServoPos, 70);
    } else if (dataBits == RIGHT) {
      set_servo_percentage(&leftServoPos, 70);
      set_servo_percentage(&rightServoPos, 110);
      set_servo_percentage(&rudderServoPos, 110);
    } else if (dataBits == UP) {
      set_servo_percentage(&leftServoPos, 110);
      set_servo_percentage(&rightServoPos, 110);
    } else if (dataBits == DOWN) {
      set_servo_percentage(&leftServoPos, 70);
      set_servo_percentage(&rightServoPos, 70);
    }
    set_activators();
  }
}




void setup() {
  // DEBUG serial setup
  Serial.begin(115200);
  
  // Servo setup
  ledcSetup(LEFT_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(LEFT_SERVO_PIN, LEFT_PWM_CHANNEL);
  set_servo_percentage(&leftServoPos, 90);
  ledcWrite(LEFT_PWM_CHANNEL, leftServoPos);
  ledcSetup(RIGHT_PWM_CHANNEL, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(RIGHT_SERVO_PIN, RIGHT_PWM_CHANNEL);
  set_servo_percentage(&rightServoPos, 90);
  ledcWrite(RIGHT_PWM_CHANNEL, rightServoPos);
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

  // Pin setup
  pinMode(RECV_PIN, INPUT);
  pinMode(LIGHTS_PIN, OUTPUT);

  // Initialise pins
  lightsToggle = 0;
  digitalWrite(LIGHTS_PIN, lightsToggle);

  // Redundant call as everything is set to still above
  sub_stand_still();

  // Communications setup
  dataBits = 0;
  nowTime = 0;
}


void loop() {
//  if (!digitalRead(RECV_PIN)) {
//    recv_signal();
//    Serial.print("got something ");
//    Serial.println(dataBits);
//    set_state();
//  }
//  delay(1);
  for (int i = 0; i < 8; i++) {
    dataBits = i;
    Serial.println(dataBits);
    set_state();
    delay(1000);
  }
}
