// Original library source: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// The default I2C address of the PCA9685 board is 0x40
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define RACKNPIN_NUM 0
#define TOP_NUM 1
#define BOTTOM_NUM 2
#define BOTH_NUM -1
#define WHEEL1_NUM 3
#define WHEEL2_NUM 4
#define ACTION_STOP 0
#define ACTION_OPEN 1
#define ACTION_CLOSE 2
#define DIR_UP 2
#define DIR_DOWN 1
#define DIR_LEFT 1
#define DIR_RIGHT 2
#define WINCHDEADZONE 1700 + 20
#define RACKDEADZONE 1130 + 10
#define WHEELDEADZONE 1685 

//ultrasonic sensor 
const int trigPin = 5;
const int echoPin = 18;
#define SOUND_SPEED 0.034
long duration;
float distanceCm;

//other variables
int steps_to_go_up = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("32 channel Servo test!");

  board1.begin();
  board1.setPWMFreq(50); //(60);  // Analog servos run at ~60 Hz updates
  //yield();

  //ultrasonic sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input

  //initialise Noboru
  init_noboru();
}

void gripper(int servo_num, int action, int ms) {
  /*
    action = 0 -> stop, action = 1 -> open, action = 2 -> close
    servo_num = 1 -> top, servo_num = 2 -> bottom
    servo_num = -1 -> both top and bottom
  */

  //int pulse = action ? 1650 : 1750; // deadzone at 1700
  int pulse = !action ? WINCHDEADZONE : ((action == ACTION_OPEN) ? (WINCHDEADZONE - 50) : (WINCHDEADZONE + 50)); 
  if (servo_num == BOTH_NUM) {
    board1.writeMicroseconds(TOP_NUM, pulse);
    board1.writeMicroseconds(BOTTOM_NUM, pulse);
    delay(ms);
    board1.writeMicroseconds(TOP_NUM, WINCHDEADZONE);
    board1.writeMicroseconds(BOTTOM_NUM, WINCHDEADZONE);
  } else {
    board1.writeMicroseconds(servo_num, pulse);
    delay(ms);
    board1.writeMicroseconds(servo_num, WINCHDEADZONE);
  }
}

void vertical_move(int direction, int ms) {
  /*
    direction = 0 -> stop
    direction = 1 -> bottom gripper moves up w.r.t top gripper
    direction = 2 -> bottom gripper moves down w.r.t top gripper
  */
 
  int pulse = !direction ? RACKDEADZONE : ((direction == DIR_DOWN) ? (RACKDEADZONE - 50) : (RACKDEADZONE + 50)); 
  board1.writeMicroseconds(RACKNPIN_NUM, pulse);
  delay(ms);
  board1.writeMicroseconds(RACKNPIN_NUM, RACKDEADZONE);
  Serial.println("stopping!");
}

void horizontal_move(int direction, int ms) {
  /*
    direction = 0 -> stop, direction = 1 -> left, direction = 2 -> right
    ms = time in milliseconds to move in the given direction
    ms = -1 -> move indefinitely in the given direction
  */
 
  int pulse = !direction ? WHEELDEADZONE : ((direction == DIR_LEFT) ? (WHEELDEADZONE - 100) : (WHEELDEADZONE + 100)); 
  board1.writeMicroseconds(WHEEL1_NUM, pulse);
  board1.writeMicroseconds(WHEEL2_NUM, pulse);
  if (ms == -1) return;
  delay(ms);
  board1.writeMicroseconds(WHEEL1_NUM, WHEELDEADZONE);
  board1.writeMicroseconds(WHEEL2_NUM, WHEELDEADZONE);
}

void init_noboru() {
  /*
    Initialise the robot noboru to grab the tree trunk.
    You have 5 seconds to grab the tree 
    trunk once both grippers are open.
  */
  gripper(BOTH_NUM, ACTION_OPEN,  15000); // 15 seconds to open both grippers
  gripper(BOTH_NUM, ACTION_STOP,  5000);  // 5 seconds still
  gripper(BOTH_NUM, ACTION_CLOSE, 15000); // 15 seconds to close both grippers
}

void stepup() {
  /*
    A single step climb up the tree.
    Only activate after noboru is initialised.
  */
  gripper(TOP_NUM, ACTION_OPEN, 20000);  
  gripper(TOP_NUM, ACTION_STOP, 5000); //get rid of this line?
  vertical_move(DIR_DOWN, 10000);
  gripper(TOP_NUM, ACTION_CLOSE, 20000);

  gripper(BOTTOM_NUM, ACTION_OPEN, 20000);  
  gripper(BOTTOM_NUM, ACTION_STOP, 5000); //get rid of this line?
  vertical_move(DIR_UP, 10000);
  gripper(BOTTOM_NUM, ACTION_CLOSE, 20000);
}

void stepdown() {
  // A single step downwards on tree.
  gripper(BOTTOM_NUM, ACTION_OPEN, 20000);  
  gripper(BOTTOM_NUM, ACTION_STOP, 5000); //get rid of this line?
  vertical_move(DIR_DOWN, 10000);
  gripper(BOTTOM_NUM, ACTION_CLOSE, 20000);

  gripper(TOP_NUM, ACTION_OPEN,  20000);  
  gripper(TOP_NUM, ACTION_STOP,  5000); //get rid of this line?
  vertical_move(DIR_UP, 10000);
  gripper(TOP_NUM, ACTION_CLOSE, 20000);
}

void scan() {
  // Scan the tree for branches.
}

void avoid_branch() {
  // Move the robot to avoid a branch.
  //scan()
  //horizontal_move(DIR_LEFT, 5000);
  //horizontal_move(DIR_RIGHT, 5000);
}

bool canopy_close(){
  //use the ultrasonic distance sensor to see if the robot reached the top of the tree
  
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  distanceCm = duration * SOUND_SPEED/2;

  //check if that is close enough to the canopy
  if (distanceCm <= 15){
    return true;
  } else {
    return false;
  }
}

void loop() {
  delay(5000);
  
  while (!canopy_close) { //go up
    stepup();
    steps_to_go_up++; //keep track of how much we go up
  }

  for (int i=0; i<steps_to_go_up; i++){ //go down
    stepdown();
  }

  //done
  delay(5000);
  init_noboru(); //grab off the tree
  delay(120000); //2min to turn off Noboru (or else it climbs again)
}
