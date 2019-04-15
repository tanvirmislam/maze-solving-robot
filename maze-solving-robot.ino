#include <NewPing.h>              // Ultrasonic sensor
#include <AFMotor.h>              // Motor
#include <PID_v1.h>               // PID

// Ultrasonic sensor pins
#define SRIGHT_TRIG_PIN       A0 
#define SRIGHT_ECHO_PIN       A1

#define SLEFT_TRIG_PIN        A2 
#define SLEFT_ECHO_PIN        A3

#define SFRONT_TRIG_PIN       A4 
#define SFRONT_ECHO_PIN       A5

// Speed constants
#define MAX_DISTANCE          200 

#define MAX_SPEED             150
#define MIN_SPEED             60

#define FORWARD_SPEED_OFFSET  30

#define TURN_ASSIST_SPEED     120
#define TURN_SPEED            80


// Define ultrasonic sensor objects
NewPing sonarRight(SRIGHT_TRIG_PIN, SRIGHT_ECHO_PIN, MAX_DISTANCE); 
NewPing sonarLeft (SLEFT_TRIG_PIN , SLEFT_ECHO_PIN , MAX_DISTANCE);
NewPing sonarFront(SFRONT_TRIG_PIN, SFRONT_ECHO_PIN, MAX_DISTANCE);

// Define motor sensor objects
AF_DCMotor motorLeft (1, MOTOR12_1KHZ);
AF_DCMotor motorRight(2, MOTOR12_1KHZ);


// Variables
int     rightDistance;
int     leftDistance;
int     frontDistance;

double  sideDifference;

int     speedSet;
int     speedIncrement;

int     leftMotorSpeed;
int     rightMotorSpeed;

boolean isMovingForward;


// PID Parameters
double  pidOutput;

double  pidSetPoint;
double  Kp = 2;
double  Ki = 5;
double  Kd = 1;

PID forwardPID( &sideDifference, 
                &pidOutput, 
                &pidSetPoint, 
                Kp, Ki, Kd, 
                DIRECT  );


void setup() {
  // Serial monitor
  Serial.begin(9600);

  // Distance variables
  readDistance();

  pidSetPoint = 0;                                        // PID's goal to reach
  forwardPID.SetOutputLimits(0, FORWARD_SPEED_OFFSET);    // PID output's range
  forwardPID.SetMode(AUTOMATIC);                          // Turn on PID

  // Speed parameters
  speedSet        = 0;
  speedIncrement  = 10;

  // State parameters
  isMovingForward   = false;
  
}


void loop() {
  delay (1000);
  
  readDistance();
  forwardPID.Compute();
  
  moveForward();

  printStatus();
}



//----------------------------------------------------
// Maze follower function
//----------------------------------------------------

void followMaze() {
    
}




//----------------------------------------------------
// Read distance using ultrasonic sensor
//----------------------------------------------------

void readDistance() {
  leftDistance  = sonarLeft.ping_cm();
  rightDistance = sonarRight.ping_cm();
  frontDistance = sonarFront.ping_cm(); 

  // Distance of zero indicates max distance
  if (leftDistance  == 0) leftDistance  = MAX_DISTANCE;
  if (rightDistance == 0) rightDistance = MAX_DISTANCE;
  if (frontDistance == 0) frontDistance = MAX_DISTANCE;
  
  sideDifference = leftDistance - rightDistance;
}


//----------------------------------------------------
// Move functions
//----------------------------------------------------

// Stop moving
void moveStop() {
  motorLeft.run(RELEASE); 
  motorRight.run(RELEASE);
} 


// Move forward
void moveForward() {
  // If has just started to move forward, reset parameters
  if (!isMovingForward) {
    motorLeft.run(FORWARD);      
    motorRight.run(FORWARD);

    isMovingForward   = true;
    
    speedSet          = MIN_SPEED;
    
    leftMotorSpeed    = speedSet;
    rightMotorSpeed   = speedSet;

    delay(5);
  }
  else {
    // Increment Speed
    if (speedSet < MAX_SPEED) {
      speedSet += speedIncrement;  
    }

    // PID Adjuster
    if (sideDifference < 0) {
      // Right distance is greater than left distance
      leftMotorSpeed  = speedSet;
      rightMotorSpeed = speedSet - pidOutput;
    }
    else if (sideDifference > 0) {
      // Left distance is greater than right distance
      leftMotorSpeed  = speedSet - pidOutput;
      rightMotorSpeed = speedSet;
    }
    else {
      // Both distances are equal
      leftMotorSpeed  = speedSet;
      rightMotorSpeed = speedSet;
    }
  }

  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);
}


// Move backward
void moveBackward() {
  // If has just started to move forward, reset parameters
  if (isMovingForward) {
    motorLeft.run(BACKWARD);      
    motorRight.run(BACKWARD);

    isMovingForward   = false;
    
    speedSet          = MIN_SPEED;
    
    leftMotorSpeed    = speedSet;
    rightMotorSpeed   = speedSet;

    delay(5);
  }
  else {
    // Increment Speed
    if (speedSet < MAX_SPEED) {
      speedSet += speedIncrement;  
    }

    // PID Adjuster
    if (sideDifference < 0) {
      // Right distance is greater than left distance
      leftMotorSpeed  = speedSet - pidOutput;
      rightMotorSpeed = speedSet;
    }
    else if (sideDifference > 0) {
      // Left distance is greater than right distance
      leftMotorSpeed  = speedSet;
      rightMotorSpeed = speedSet - pidOutput;
    }
    else {
      // Both distances are equal
      leftMotorSpeed  = speedSet;
      rightMotorSpeed = speedSet;
    }
  }

  motorLeft.setSpeed(leftMotorSpeed);
  motorRight.setSpeed(rightMotorSpeed);
}  


// Turn left
void turnLeft() {
  // If has just started to move forward, reset parameters
  if (!isMovingForward) {
    motorLeft.run(FORWARD);      
    motorRight.run(FORWARD);

    isMovingForward = true;
    
    speedSet = 0;
  
    delay(5);
  }

  motorLeft.setSpeed(TURN_SPEED);
  motorRight.setSpeed(TURN_ASSIST_SPEED);
}  


// Turn right
void turnRight() {
  // If has just started to move forward, reset parameters
  if (!isMovingForward) {
    motorLeft.run(FORWARD);      
    motorRight.run(FORWARD);

    isMovingForward = true;
    
    speedSet = 0;
  
    delay(5);
  }

  motorLeft.setSpeed(TURN_ASSIST_SPEED);
  motorRight.setSpeed(TURN_SPEED);
} 
 


//----------------------------------------------------
// Print function
//----------------------------------------------------

void printStatus() {
  Serial.print("Left Distance: ");
  Serial.println(leftDistance); 
  Serial.print("Right Distance: ");
  Serial.println(rightDistance);
  Serial.print("Front Distance: ");
  Serial.println(frontDistance); 
  Serial.print("Side Difference: ");
  Serial.println(sideDifference); 
  
  Serial.print("PID Output: ");
  Serial.println(pidOutput);

  Serial.println("Current speed: ");
  Serial.println(speedSet);
  Serial.println("Left motor speed: ");
  Serial.println(leftMotorSpeed);
  Serial.println("Right motor speed: ");
  Serial.println(rightMotorSpeed);
  Serial.println();
}
