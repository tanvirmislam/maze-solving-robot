
#include <NewPing.h> 
#include "GYRO.h"


//***************************************************************************************************************************
// Variables
//***************************************************************************************************************************

//---------------------------------------------------------------------
// State of the robot
//---------------------------------------------------------------------
enum STATE {
    ONLY_FRONT_OPEN,
    ONLY_LEFT_OPEN,
    ONLY_RIGHT_OPEN,
    LEFT_AND_FRONT_OPEN,
    RIGHT_AND_FRONT_OPEN,
    LEFT_AND_RIGHT_OPEN,
    DEAD_END,
    OPEN
};


STATE currentState;
STATE previousState;



//---------------------------------------------------------------------
// Ultrasonic sensor pins
//---------------------------------------------------------------------
#define SLEFT_TRIG_PIN      A0
#define SLEFT_ECHO_PIN      A1
 
#define SRIGHT_TRIG_PIN     A2
#define SRIGHT_ECHO_PIN     A3
 
#define SFRONT_TRIG_PIN     A4
#define SFRONT_ECHO_PIN     A5

//---------------------------------------------------------------------
// Distance constants
//---------------------------------------------------------------------
#define MAX_DISTANCE                    100
#define STOPPING_DISTANCE               20    // Stop if front sensor reads a distance less than this
#define WALL_VISIBILITY_THRESHOLD       30
#define SIDE_DIFFERENCE_THRESHOLD       5

//---------------------------------------------------------------------
// Define ultrasonic sensor objects
//---------------------------------------------------------------------
NewPing sonarRight(SRIGHT_TRIG_PIN, SRIGHT_ECHO_PIN, MAX_DISTANCE);
NewPing sonarLeft (SLEFT_TRIG_PIN , SLEFT_ECHO_PIN , MAX_DISTANCE);
NewPing sonarFront(SFRONT_TRIG_PIN, SFRONT_ECHO_PIN, MAX_DISTANCE);

//---------------------------------------------------------------------
// Distance Variables
//---------------------------------------------------------------------
int     rightDistance;
int     leftDistance;
int     frontDistance;
double  sideDifference;

boolean isObstacleDetected;
boolean isFrontWallVisible;
boolean isLeftWallVisible;
boolean isRightWallVisible;




//---------------------------------------------------------------------
// Motor Variables
//---------------------------------------------------------------------
#define STRAIGHT_PATH_SPEED             100

#define SLIGHT_TURN_ASSIST_SPEED        90
#define SLIGHT_TURN_SPEED               75
#define SLIGHT_TURN_DURATION            100
 
#define HARSH_TURN_ASSIST_SPEED         120
#define HARSH_TURN_SPEED                60

#define ADJUST_PATH_ASSIST_SPEED        105
#define ADJUST_PATH_SPEED               75
#define ADJUST_PATH_DURATION            500

// For new battery
#define NINETY_DEGREE_LEFT_CM           15.50
#define NINETY_DEGREE_RIGHT_CM          14.00


const byte MOTOR_RIGHT = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_LEFT  = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Constant for steps in disk
const float stepcount = 20.00;  // 20 Slots in disk

// Constant for wheel diameter
const float wheeldiameter = 64; // Wheel diameter in millimeters

// Integers for pulse counters
volatile int counter_right = 0;
volatile int counter_left  = 0;

    
// LEFT MOTOR
int enA = 10;
int in1 = 9;
int in2 = 8;

// RIGHT RIGHT
int enB = 5;
int in3 = 7;
int in4 = 6;


// Right motor seems to spin slower than left motor
// Increasing PWM by 20 makes them spin almost identically
int delta_left  = 0;
int delta_right = 20;


boolean isMovingForward;


//---------------------------------------------------------------------
// Gyro Reading
//---------------------------------------------------------------------
// Angle
float current_angle = 0.0;




//***************************************************************************************************************************
// Setup
//***************************************************************************************************************************

void setup() {
    delay(5000);
    
    // Serial monitor
    //Serial.begin(9600);

    // Attach the Interrupts to their ISR's
    attachInterrupt(digitalPinToInterrupt (MOTOR_RIGHT), ISR_count_right, RISING);  // Increase counter A when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt (MOTOR_LEFT),  ISR_count_left,  RISING);  // Increase counter B when speed sensor pin goes High

    // Start reading distances
    for (int i = 0; i < 5; i++) {
        readDistance();
        detectState();
    }
    
    previousState = currentState;

    //initialize_gyro();
    //current_angle = getAngle();

    isMovingForward = false;
} 




//***************************************************************************************************************************
// Loop
//***************************************************************************************************************************

void loop() {
    readDistance();
    detectState();
    navigateMaze();

    //turnRightNinetyDegrees();
    //pause(2000);
    
    //turnRightNinetyDegrees();
    //uTurn();
    
    //printState();
    
    //pause(2000);

    //turnLeftNinetyDegrees();

    //pause(2000);
}


                         
//***************************************************************************************************************************
// Ultrasonic Sensor Functions
//***************************************************************************************************************************

//---------------------------------------------------------------------
// Read distance using ultrasound sensor
//---------------------------------------------------------------------
void readDistance() {
    leftDistance  =  sonarLeft.ping_cm() ;
    rightDistance =  sonarRight.ping_cm();
    frontDistance =  sonarFront.ping_cm();

    leftDistance  =  sonarLeft.ping_cm() ;
    rightDistance =  sonarRight.ping_cm();
    frontDistance =  sonarFront.ping_cm();
    
    // Distance of zero indicates max distance
    if (leftDistance  == 0) leftDistance  = MAX_DISTANCE;
    if (rightDistance == 0) rightDistance = MAX_DISTANCE;
    if (frontDistance == 0) frontDistance = MAX_DISTANCE;
    
    sideDifference = leftDistance - rightDistance;
    
    if (frontDistance <= STOPPING_DISTANCE) isObstacleDetected = true;
    else isObstacleDetected = false;  
    
    if (frontDistance <= WALL_VISIBILITY_THRESHOLD) isFrontWallVisible = true;
    else isFrontWallVisible = false;
    
    if (leftDistance <= WALL_VISIBILITY_THRESHOLD) isLeftWallVisible = true;
    else isLeftWallVisible = false;
    
    if (rightDistance <= WALL_VISIBILITY_THRESHOLD) isRightWallVisible = true;
    else isRightWallVisible = false;
}



//***************************************************************************************************************************
// State Detection
//***************************************************************************************************************************

//----------------------------------------------------
// Detect state
//----------------------------------------------------
void detectState() {
    if (isObstacleDetected == true && isLeftWallVisible == true && isRightWallVisible == true) {
        currentState = DEAD_END;
    }
    else if (isObstacleDetected == true && isLeftWallVisible == false && isRightWallVisible == false) {
        currentState = LEFT_AND_RIGHT_OPEN;  
    }
    else if (isObstacleDetected == false && isLeftWallVisible == true && isRightWallVisible == true) {
        currentState = ONLY_FRONT_OPEN;
    }
    else if (isObstacleDetected == true && isLeftWallVisible == false && isRightWallVisible == true) {
        currentState = ONLY_LEFT_OPEN;  
    }
    else if (isObstacleDetected == true && isLeftWallVisible == true && isRightWallVisible == false) {
        currentState = ONLY_RIGHT_OPEN;  
    }
    else if (isFrontWallVisible == false && isLeftWallVisible == false && isRightWallVisible == true) {
        currentState = LEFT_AND_FRONT_OPEN;
    }
    else if (isFrontWallVisible == false && isLeftWallVisible == true && isRightWallVisible == false) {
        currentState = RIGHT_AND_FRONT_OPEN;  
    }
    else {
        currentState = OPEN;  
    }
}




//***************************************************************************************************************************
// Interrupt Service Routines
//***************************************************************************************************************************

//---------------------------------------------------------------------
// Motor B pulse count ISR
//---------------------------------------------------------------------
void ISR_count_left()   {
    counter_left++;   // increment LEFT MOTOR counter value
}

//---------------------------------------------------------------------
// Motor A pulse count ISR
//---------------------------------------------------------------------
void ISR_count_right() {
    counter_right++;  // increment RIGHT MOTOR counter value
} 


//***************************************************************************************************************************
// Conversion from CM to STEPS
//***************************************************************************************************************************

//---------------------------------------------------------------------
// Function to convert from centimeters to steps
//---------------------------------------------------------------------
int cmToSteps(float cm) {
    int result;                                             // Final calculation result
    float circumference = (wheeldiameter * 3.14) / 10;      // Calculate wheel circumference in cm
    float cm_step = circumference / stepcount;              // CM per Step
    
    float f_result = cm / cm_step;                          // Calculate result as a float
    result = (int) f_result;                                // Convert to an integer (note this is NOT rounded)
    
    return result;                                          // End and return result

}




//***************************************************************************************************************************
// Motor Functions
//***************************************************************************************************************************

//---------------------------------------------------------------------
// Function to Move Forward Indefinitey with Given Speed
//---------------------------------------------------------------------
boolean initMove() {
    if (!isMovingForward) {
        // Set LEFT MOTOR forward
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    
        // Set RIGHT MOTOR forward
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);       

        isMovingForward = true;

        return true;
    }

    return false;
}

void moveForward(int lm_spd, int rm_spd) {
    boolean hasInit = initMove();

    //if (hasInit) {
    //    analogWrite(enA, 200);
    //    analogWrite(enB, 200);
    //    delay(2);
    //}
    
    analogWrite(enA, lm_spd + delta_left);
    analogWrite(enB, rm_spd + delta_right);
}

//---------------------------------------------------------------------
// Function to Move Forward
//---------------------------------------------------------------------
void stepForward(int steps, int lm_speed, int rm_speed) {
    initMove();
    
    counter_right = 0;  //  reset counter right to zero
    counter_left  = 0;  //  reset counter left  to zero
   
    // Go forward until step value is reached
    while (steps > counter_left && steps > counter_right) {
        // Check left
        if (steps > counter_left) {
            analogWrite(enA, lm_speed + delta_left);
        }
        else {
            analogWrite(enA, 0);
        }

        // Check right
        if (steps > counter_right) {
            analogWrite(enB, rm_speed + delta_right);
        } 
        else {
            analogWrite(enB, 0);
        }
    
    }
    
    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    
    counter_left  = 0;  //  reset counter A to zero
    counter_right = 0;  //  reset counter B to zero 
}


//---------------------------------------------------------------------
// Function to Spin Left
//---------------------------------------------------------------------
void spinLeftWithBothWheels(int steps, int mspeed) {
    counter_left  = 0;  //  reset counter left to zero
    counter_right = 0;  //  reset counter right to zero
   
    // Set LEFT MOTOR reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // Set RIGHT MOTOR forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
   
    isMovingForward = false;
    
    // Go until step value is reached
    while (steps > counter_left && steps > counter_right) {
        // Check left
        if (steps > counter_left) {
            analogWrite(enA, mspeed + delta_left);
        } 
        else {
            analogWrite(enA, 0);
        }

        // Check right
        if (steps > counter_right) {
            analogWrite(enB, mspeed + delta_right);
        } 
        else {
            analogWrite(enB, 0);
        }
    }
    
    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    
    counter_left  = 0;  //  reset counter A to zero
    counter_right = 0;  //  reset counter B to zero 

}


//---------------------------------------------------------------------
// Function to Spin Left
//---------------------------------------------------------------------
void spinLeftWithOneWheel(int steps, int mspeed) {
    counter_left  = 0;  //  reset counter left to zero
    counter_right = 0;  //  reset counter right to zero
    
    // Set LEFT MOTOR reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    
    // Set RIGHT MOTOR reverse
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    // Left motor is not moving
    analogWrite(enA, 0);

    isMovingForward = false;
    
    // Go until step value is reached
    while (steps > counter_right) {
        // Check right
        if (steps > counter_right) {
            analogWrite(enB, mspeed + delta_right);
        } 
        else {
            analogWrite(enB, 0);
        }
    }
    
    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    
    counter_left  = 0;  //  reset counter A to zero
    counter_right = 0;  //  reset counter B to zero 

}


//---------------------------------------------------------------------
// Function to Spin Right
//---------------------------------------------------------------------
void spinRightWithBothWheels(int steps, int mspeed) {
    counter_left  = 0;  //  reset counter left to zero
    counter_right = 0;  //  reset counter right to zero
   
    // Set LEFT MOTOR forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // Set RIGHT MOTOR reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    isMovingForward = false;
    
    // Go until step value is reached
    while (steps > counter_left && steps > counter_right) {
        // Check left
        if (steps > counter_left) {
            analogWrite(enA, mspeed + delta_left);
        } 
        else {
            analogWrite(enA, 0);
        }

        // Check right
        if (steps > counter_right) {
            analogWrite(enB, mspeed + delta_right);
        } 
        else {
            analogWrite(enB, 0);
        }
    }
    
    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    
    counter_left  = 0;  //  reset counter A to zero
    counter_right = 0;  //  reset counter B to zero 
}


//---------------------------------------------------------------------
// Function to Spin Right
//---------------------------------------------------------------------
void spinRightWithOneWheel(int steps, int mspeed) {
    counter_left  = 0;  //  reset counter left to zero
    counter_right = 0;  //  reset counter right to zero
   
    // Set LEFT MOTOR forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // Set RIGHT MOTOR reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    // Righ motor is not moving
    analogWrite(enB, 0);

    isMovingForward = false;
    
    // Go until step value is reached
    while (steps > counter_left) {
        // Check left
        if (steps > counter_left) {
            analogWrite(enA, mspeed + delta_left);
        } 
        else {
            analogWrite(enA, 0);
        }
    }
    
    // Stop when done
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    
    counter_left  = 0;  //  reset counter A to zero
    counter_right = 0;  //  reset counter B to zero 
}



//---------------------------------------------------------------------
// Stop Moving
//---------------------------------------------------------------------
void moveStop() {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    isMovingForward = false;
}



//***************************************************************************************************************************
// Navigation Functions
//***************************************************************************************************************************


//---------------------------------------------------------------------
// Stop move for given ms
//---------------------------------------------------------------------
void pause(int dur) {
    moveStop();
    delay(dur);
}


//---------------------------------------------------------------------
// Navigate straight
//---------------------------------------------------------------------
void navigateStraight() {
    initMove();
    
    if (sideDifference < 0) {
        moveForward(STRAIGHT_PATH_SPEED + 10, STRAIGHT_PATH_SPEED - 10); 
        delay(SLIGHT_TURN_DURATION);
    }
    else if (sideDifference > 0) {
        moveForward(STRAIGHT_PATH_SPEED - 10, STRAIGHT_PATH_SPEED + 10); 
        delay(SLIGHT_TURN_DURATION);
    }

    moveForward(STRAIGHT_PATH_SPEED, STRAIGHT_PATH_SPEED); 
}


//---------------------------------------------------------------------
// Navigate straight for a given amount of time
//---------------------------------------------------------------------
void navigateStraight(int dur) {
    navigateStraight();
    delay(dur);
    moveStop();
}


//---------------------------------------------------------------------
// Take left path
//---------------------------------------------------------------------
void takeLeftPath() {
    pause(1000);
    turnLeftNinetyDegrees();
    isMovingForward = false;
    pause(1000);
    navigateStraight(400);
    //pause(1000);
}

//---------------------------------------------------------------------
// Take right path
//---------------------------------------------------------------------
void takeRightPath() {
    pause(1000);
    turnRightNinetyDegrees();
    isMovingForward = false;
    pause(2000);
    navigateStraight(400);
    //pause(1000);
}


//---------------------------------------------------------------------
// Spin left ninety degrees
//---------------------------------------------------------------------
void turnLeftNinetyDegrees() {
    // spinLeftWithBothWheels(8,94);
    spinLeftWithOneWheel(cmToSteps(NINETY_DEGREE_LEFT_CM), 95);       
}

//---------------------------------------------------------------------
// Spin right ninety degrees
//---------------------------------------------------------------------
void turnRightNinetyDegrees() {
    // spinRightWithBothWheels(8, 96);    
    spinRightWithOneWheel(cmToSteps(NINETY_DEGREE_RIGHT_CM), 95);
}


//---------------------------------------------------------------------
// U Turn
//---------------------------------------------------------------------
void uTurn() {
    pause(2000);
    //turnLeftNinetyDegrees();
    spinRightWithBothWheels(14,91);    
    pause(2000);
}


//---------------------------------------------------------------------
// Turn left for adjutment
//---------------------------------------------------------------------
//void turnLeftToAdjustPath(int lm_speed, int rm_speed) {
//    // Set LEFT MOTOR reverse
//    digitalWrite(in1, LOW);
//    digitalWrite(in2, HIGH);
//
//     Set RIGHT MOTOR forward
//    digitalWrite(in3, HIGH);
//    digitalWrite(in4, LOW); 
//
//    analogWrite(enA, lm_speed + delta_left);
//    analogWrite(enB, rm_speed + delta_right);
//}


//---------------------------------------------------------------------
// Turn right for adjutment
//---------------------------------------------------------------------
//void turnRightToAdjustPath(int lm_speed, int rm_speed) {
//    // Set LEFT MOTOR reverse
//    digitalWrite(in1, HIGH);
//    digitalWrite(in2, LOW);
//
//     Set RIGHT MOTOR forward
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, HIGH); 
//
//    analogWrite(enA, lm_speed + delta_left);
//    analogWrite(enB, rm_speed + delta_right);
//}



//***************************************************************************************************************************
// Print Functions
//***************************************************************************************************************************

//---------------------------------------------------------------------
// Print current state
//---------------------------------------------------------------------
void printState() {
    switch (currentState) {
        case ONLY_FRONT_OPEN:
            Serial.println("State: Only Front Open");
            break;
        case ONLY_LEFT_OPEN:
            Serial.println("State: Only Left Open");
            break;
        case ONLY_RIGHT_OPEN:
            Serial.println("State: Only Right Open");
            break;
        case LEFT_AND_FRONT_OPEN:
            Serial.println("State: Left and Front Open");
            break;
        case RIGHT_AND_FRONT_OPEN:
            Serial.println("State: Right and Front Open");
            break;
        case LEFT_AND_RIGHT_OPEN:
            Serial.println("State: Left and Right Open");
            break;
        case DEAD_END:
            Serial.println("State: Dead End");
            break;
        case OPEN:
            Serial.println("State: Open");
            break;
    }
}
  
//---------------------------------------------------------------------
// Print distances
//---------------------------------------------------------------------
void printDistances() {
    Serial.print("Left Distance: ");
    Serial.println(leftDistance);
    Serial.print("Right Distance: ");
    Serial.println(rightDistance);
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    Serial.print("Side Difference: ");
    Serial.println(sideDifference);    
    Serial.println();  
}


//---------------------------------------------------------------------
// Print encoder readings
//---------------------------------------------------------------------
void printEncoderCounter() {
    Serial.print("Left count:  ");
    Serial.println(counter_left);
    Serial.print("Right count: ");
    Serial.println(counter_right);
    Serial.println();        
}




//***************************************************************************************************************************
// MAZE SOLVER
//***************************************************************************************************************************
void navigateMaze() {
    switch (currentState) {
        case LEFT_AND_RIGHT_OPEN:
            takeLeftPath();
            break;
        case ONLY_FRONT_OPEN:
            navigateStraight();
            break;
        case ONLY_LEFT_OPEN:
            takeLeftPath();
            break;
        case ONLY_RIGHT_OPEN:
            takeRightPath();
            break;
        case LEFT_AND_FRONT_OPEN:
            takeLeftPath();
            break;
        case RIGHT_AND_FRONT_OPEN:
            navigateStraight();
            break;  
        case DEAD_END:
            uTurn();
            break;
        case OPEN:
            
            break;
    }
    
}
