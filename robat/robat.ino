/**
 *
 * RoBat - Robot Construction Kit
 *
 * Michael Ibsen
 * ibsen@gmx.net
 *
 * 2018-05-25
 *
 * License: GNU GPLv3
 *
 *
 */

// --- BEGIN INIT GENERAL ---

#define DEBUG false

// sleep duration
// #define SLEEP_INTERVAL 10000

// --- END INIT GENERAL ---


// --- BEGIN INIT AUTOMANUAL ---
#define MANUAL 0
#define AUTONOMOUS 1
#define BATTLE 2

// enum Color: byte
/*
enum Mode: int
{
   MANUAL, AUTONOMOUS, BATTLE
};
*/

// default mode: manual
int nMode = MANUAL;

// Zustand des Endlichen Automaten
int nBattleState = 0;
int nAttackCounter = 0;

unsigned long timeOfLastAction = 0;

// --- BEGIN INIT AUTOMANUAL ---


// --- BEGIN INIT BUMPER ---

#define BUMPER_PIN 2

// --- END INIT BUMPER ---


// --- BEGIN INIT BUZZER ---

#include <TimerFreeTone.h>

// Set up speaker on a PWM pin (digital 9, 10 or 11)
#define TONE_PIN 11 // Pin you have speaker/piezo connected to (be sure to include a 100 ohm resistor).

#define NOTE_A4 220
#define NOTE_A 440
#define NOTE_CIS 554
#define NOTE_D 587
#define NOTE_E 659
#define NOTE_FIS 740
#define NOTE_G 784
#define NOTE_G6 831
#define NOTE_A6 880
#define NOTE_PAUSE 0

int melody[] = { 262, 196, 196, 220, 196, 0, 247, 262 };
int duration[] = { 250, 125, 125, 250, 250, 250, 250, 250 };

/*
// jurassic park
// D - D c# D - D c# D - E E GG f# D E  c# A  f# D E  ADG  f# f# E E
#define melody_length 36
int melody[] = {
  NOTE_D, NOTE_PAUSE,
  NOTE_D, NOTE_CIS, NOTE_D, NOTE_PAUSE,
  NOTE_D, NOTE_CIS, NOTE_D, NOTE_PAUSE,
  NOTE_E, NOTE_E, NOTE_PAUSE, NOTE_G, NOTE_G, NOTE_PAUSE,
  NOTE_FIS, NOTE_D, NOTE_E, NOTE_PAUSE, NOTE_CIS, NOTE_A, NOTE_PAUSE,
  NOTE_FIS, NOTE_D, NOTE_E, NOTE_PAUSE,
  NOTE_A6, NOTE_D, NOTE_G, NOTE_PAUSE, NOTE_FIS, NOTE_FIS, NOTE_PAUSE, NOTE_E, NOTE_E };
int duration[] = {
  500, 250,
  250, 250, 500, 250,
  250, 250, 500, 250,
  250, 500, 125, 250, 500, 125,
  250, 250, 500, 125, 250, 500, 125,
  250, 250, 500, 250,
  250, 250, 500, 125, 250, 500, 125, 250, 500
  };
*/

// --- END INIT BUZZER ---


// --- BEGIN INIT SERVO ---

#include <Servo.h>

// servos on A0 (D14) and A1 (D15)
#define SERVO_1_PIN 12
#define SERVO_1_MIN 25 // Fine tune your servos min. 0-180
#define SERVO_1_MAX 180  // Fine tune your servos max. 0-180
#define SERVO_2_PIN 13
#define SERVO_2_MIN 10 // Fine tune your servos min. 0-180
#define SERVO_2_MAX 170  // Fine tune your servos max. 0-180
#define DETACH_DELAY 250 // Tune this to let your movement finish before detaching the servo

// create servo objects to control the servos
Servo servo2;
Servo servo1;

bool bAttachedServo2 = false;
bool bAttachedServo1 = false;

// time of last change
unsigned long timeOfLastChangeServo2 = 0;
unsigned long timeOfLastChangeServo1 = 0;

// servo position
int actualPositionServo2 = 0;
int targetPositionServo2 = 0;
int actualPositionServo1 = 0;
int targetPositionServo1 = 0;

// --- END INIT SERVO ---


// --- BEGIN INIT ULTRASONIC ---

#define TRIGGER_PIN  15  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     14  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#include <Ultrasonic.h>

boolean bMeasureDistance = true;
unsigned long timeOfLastDistanceMeasurement = millis();

// time between distance measurements
#define DISTANCE_DELAY 2000

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
// ultrasonic.setTimeout(10000UL); // max. 40000UL

// --- END INIT ULTRASONIC ---


// --- BEGIN INIT WS2811 ---

#define FASTLED_ESP8266_RAW_PIN_ORDER
//#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
//#define FASTLED_ESP8266_D1_PIN_ORDER

#include <FastLED.h>

// number of LEDs in the strip
#define NUM_LEDS 12

// the data pin the green wire from the LEDs are connected to
#define WS2811_PIN 10

// an array to keep track of the LEDs
CRGB leds[NUM_LEDS];

// how many colors to cycle through for the lights
// #define NUM_COLORS 4
int num_colors = 24;

// --- END INIT WS2811 ---

// --- BEGIN INIT MOTOR ---

// Motor A
int enA = 6; // 9
int in1 = 7;
int in2 = 8;

// Motor B
int enB = 3;
int in3 = 5;
int in4 = 4;

int stby = 9; // 6

// Motor Speed Values - Start at zero
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

// MAX_SPEED 0..255
#define MAX_SPEED 196

// motor A / B, aka left / right
#define MOTOR_A 0
#define MOTOR_B 1

// backward / forward
#define FORWARD 0
#define BACKWARD 1

// left / right
#define LEFT 0
#define RIGHT 1

// --- END INIT MOTOR ---

// --- BEGIN INIT JOYSTICK ---

// Joystick Input
int joyVert = A6; // Vertical
int joyHorz = A7; // Horizontal

// Joystick Values - Start at 512 (middle position)
int joyposVert = 512;
int joyposHorz = 512;

// --- END INIT JOYSTICK ---


// --- BEGIN INIT OBSTACLE-AVOIDANCE ---

// --- END INIT OBSTACLE-AVOIDANCE ---


// --- BEGIN FUNCTIONS ULTRASONIC ---

/**
 *  Reads distance from ultrasonic sensor several times and calculates
 *  the average value. Negative distances are ignored.
 *
 *  @param int nMeasurements: number of measurements, default 5
 *  @return int nDistanceAvg: average distance
 *
 */
int getDistance(int nMeasurements = 5) {
  int nDistanceAvg = 0;

  int nDistance = 0;
  int nDistanceSum = 0;
  int nValidValues = 0; // number of positive values

  for (int i=0; i<=nMeasurements; i++) {
    nDistance = ultrasonic.distanceRead(CM);
    if (nDistance > 0) {
      nDistanceSum += nDistance;
      nValidValues++;
    }
  }

  if (nValidValues > 0) {
    nDistanceAvg = floor(nDistanceSum / nValidValues);
  }

  return nDistanceAvg;
}

// --- END FUNCTIONS ULTRASONIC ---


// --- BEGIN FUNCTIONS BUZZER ---

/**
 * make a happy or disappointed sound
 * @param int nMode: 0 - happy sound, 1 - disappointed sound
 *
 */
void tweet(int nPin, int nMode = 0) {
    switch(nMode) {
        case 1:
            // annoyed tweet
            for (int i=200; i<1000; i++) {
              TimerFreeTone(nPin, i, 10);
              TimerFreeTone(nPin, i, 10);
            }
            for (int i=1000; i<1500; i++) {
              TimerFreeTone(nPin, i, 5);
            }
            for (int i=1500; i>100; i--) {
              TimerFreeTone(nPin, i, 8);
            }
            break;
        case 2:
            // melody 1
            TimerFreeTone(nPin, 400, 200);
            delay(100);
            TimerFreeTone(nPin, 500, 200);
            delay(100);
            TimerFreeTone(nPin, 600, 200);
            delay(100);
            TimerFreeTone(nPin, 600, 200);
            delay(100);
            TimerFreeTone(nPin, 600, 200);
            delay(100);
            TimerFreeTone(nPin, 200, 200);
            break;
        case 3:
            // fall
            for (int i=1000; i>300; i--) {
                TimerFreeTone(nPin, i, 200);
                delay(5);
            }
            break;
        case 4:
            // melody 3 TimerFreeTone
            TimerFreeTone(nPin, 200, 300);
            delay(300);
            TimerFreeTone(nPin, 800, 200);
            delay(200);
            TimerFreeTone(nPin, 400, 500);
            delay(500);

            break;
        case 5:
            // rise
            for (int i=300; i<750; i++) {
                TimerFreeTone(nPin, i, 10);
                delay(2);
            }
            break;
        case 0:
        default:
            // happy tweet

            for (int i=500; i<1000; i++) {
                TimerFreeTone(nPin, i, 100);
            }
            for (int i=1000; i<2000; i++) {
                TimerFreeTone(nPin, i, 50);
            }
    }

}

// --- END FUNCTIONS BUZZER ---


// --- BEGIN FUNCTIONS LED ---

/**
 * Uses the LED ring to show the distance to the nearest object(s).
 *
 * LED 6 shows the distance at an angle of 60 degrees
 * LED 7 shows the distance at an angle of 90 degrees
 * LED 8 shows the distance at an angle of 130 degrees
 *
 * @param int nAngle angle
 * @param int nDistance distance to object in this direction
 *
 */
void showDistance(int nAngle, int nDistance) {

    /*
    if (DEBUG) {
      Serial.print("angle: ");
      Serial.println(nAngle);
      Serial.print("distance: ");
      Serial.print(nDistance);
    }
    */

    // color of LED depends on the distance:
    // green: "very far", greater than 50cm
    // blue: 20 cm > distance <= 50cm
    // purple: 10 cm > distance <= 20cm
    // red: < 10 cm

    CRGB color = CRGB::Black;
    if (nDistance > 150) {
      color = CRGB::Black;
    }
    else if ((nDistance <= 150) && (nDistance > 50)) {
      color = CRGB::Red; // green
    }
    else if ((nDistance <= 60) && (nDistance > 30)) {
      color = CRGB::Blue;
    }
    else if ((nDistance <= 30) && (nDistance > 15)) {
      color = CRGB::DarkTurquoise;
    }
    else if (nDistance > 0) {
      color = CRGB::Green; // red
    }
    else {
      color = CRGB::Black;
    }

    switch (nAngle) {
      case 60:
        leds[0] = color;
        break;
      case 90:
        leds[1] = color;
        break;
      case 130:
        leds[2] = color;
        break;
    }
    FastLED.show();
}

// --- END FUNCTIONS LED ---


// --- BEGIN FUNCTIONS SERVO ---

void startServo2(int targetPos) {
    targetPositionServo2 = targetPos;

    // start servo
    servo2.attach(SERVO_2_PIN);
    bAttachedServo2 = true;

    // sets the servo position 0-180
    servo2.write(SERVO_2_MAX + (SERVO_2_MIN - SERVO_2_MAX) / 100 * targetPositionServo2);
    // timeOfLastChangeServo2 = millis();

    if (DEBUG) {
      Serial.print("actualPositionServo2: ");
      Serial.println(actualPositionServo2);
      Serial.print("targetPositionServo2: ");
      Serial.println(targetPositionServo2);
    }

    actualPositionServo2 = -1;
}

void startServo1(int targetPos) {
    targetPositionServo1 = targetPos;

    // start servo
    servo1.attach(SERVO_1_PIN);
    bAttachedServo1 = true;

    // sets the servo position 0-180
    // constrain(x, a, b)
    // int targetPosCorrected = constrain(targetPositionServo1, SERVO_1_MIN, SERVO_1_MAX);
    // map(value, fromLow, fromHigh, toLow, toHigh)
    int targetPosCorrected = map(targetPositionServo1, 0, 180, SERVO_1_MIN, SERVO_1_MAX);
    servo1.write(targetPosCorrected);
    // timeOfLastChangeServo1 = millis();

    if (DEBUG) {
      Serial.print("actualPositionServo1: ");
      Serial.println(actualPositionServo1);
      Serial.print("targetPositionServo1: ");
      Serial.println(targetPositionServo1);
      Serial.print("targetPos (corrected): ");
      Serial.println(targetPosCorrected);
    }

    actualPositionServo1 = -1;
}

void stopServo2() {
   servo2.detach();
   bAttachedServo2 = false;
   actualPositionServo2 = targetPositionServo2;

   if (DEBUG) {
     Serial.println("stopping servo 2");
   }
}

void stopServo1() {
   servo1.detach();
   bAttachedServo1 = false;
   actualPositionServo1 = targetPositionServo1;

   if (DEBUG) {
     Serial.println("stopping servo 1");
   }
}

void moveServoBackForth() {
  // move servo back and forth

  unsigned long timeNow = millis();

  // cycle servo through 60, 90 and 130 degrees
  // initial move
  if (0 == actualPositionServo1) {
    startServo1(60);
    timeOfLastChangeServo1 = timeNow;
  }
  else if (60 == actualPositionServo1) {
    startServo1(90);
    timeOfLastChangeServo1 = timeNow;
  }
  else if (90 == actualPositionServo1) {
    startServo1(130);
    timeOfLastChangeServo1 = timeNow;
  }
  else if (130 == actualPositionServo1) {
    startServo1(60);
    timeOfLastChangeServo1 = timeNow;
  }

  // stop servos
  /*
  if (bAttachedServo2 && timeNow - timeOfLastChangeServo2 > DETACH_DELAY) {
    stopServo2();
  }
  */
  if (bAttachedServo1 && timeNow - timeOfLastChangeServo1 > DETACH_DELAY) {
    if (DEBUG) {
      Serial.print("servo time:");
      Serial.println(timeNow - timeOfLastChangeServo1);
    }
    stopServo1();

    // measure distance to next objects in front of the robot
    int nDistance = ultrasonic.distanceRead(CM);
    if (DEBUG) {
      Serial.print("Distance to object, ");
      Serial.print(actualPositionServo1);
      Serial.print(" degrees: ");
      Serial.println(nDistance);
    }

    // set LEDs to show distance
    showDistance(actualPositionServo1, nDistance);


  }
}

// --- BEGIN FUNCTIONS SERVO ---


// --- BEGIN FUNCTIONS MOTOR ---

/**
 * Start motor
 *
 * @param int nMotor: 0 - motor A, 1 - motor B
 * @param int nDir: 0 - forward, 1 - backward
 * @param int nSpeed: speed 0..255, limited by MAX_SPEED
 *
 */
void startMotor(int nMotor, int nDir, int nSpeed) {

  // limit speed to 0 .. MAX_SPEED
  nSpeed = max(0, min(nSpeed, MAX_SPEED));

  if (MOTOR_A == nMotor) {
    if (FORWARD == nDir) {
      // motor A forward
      // Set Motor A forward
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else {
      // motor A backward
      // Set Motor A backward
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    analogWrite(enA, nSpeed);
  }
  else {
    if (FORWARD == nDir) {
      // motor B forward
      // Set Motor B forward
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
    }
    else {
      // motor B backward
      // Set Motor B backward
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
    analogWrite(enB, nSpeed);
  }
}

/**
 * Start motor
 *
 * @param int nMotor: 0 - motor A, 1 - motor B
 */
void stopMotor(int nMotor) {
  if (MOTOR_A == nMotor) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0); // set high to switch motor off, 0 for short brake
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0); // set high to switch motor off, 0 for short brake
  }
}


/**
 * Turns robot left or right by starting motor A forward and motor B backward
 * or vice versa for nDelay milliseconds.
 *
 * @param int nDir: 0 - left / 1 - right
 * @param int nDelay: delay in milliseconds
 *
 */
void turnRobot(int nDir, int nDelay) {
  int nSpeed = 128;
  if (LEFT == nDir) {
    startMotor(MOTOR_A, FORWARD, nSpeed);
    startMotor(MOTOR_B, BACKWARD, nSpeed);
  }
  else {
    startMotor(MOTOR_A, BACKWARD, nSpeed);
    startMotor(MOTOR_B, FORWARD, nSpeed);
  }
  delay(nDelay); // TODO: replace by non-blocking control
  stopMotor(MOTOR_A);
  stopMotor(MOTOR_B);
}

// --- END FUNCTIONS MOTOR ---


// --- BEGIN FUNCTIONS DO_BATTLE ---

// depends on MOTOR, SERVO and ULTRASONIC

/**
 * Measures the distance to nearest object at angle nAngle
 *
 * @param int nAngle: angle
 * @return int nDistance: distance to object in cm
 *
 */
int getDistanceDir(int nAngle) {

  startServo1(nAngle);
  delay(DETACH_DELAY);
  stopServo1();
  int nDistance = getDistance();

  return nDistance;
}


/**
 * Measures the distance to the nearest object in different directions
 * and returns the angle of the direction to the nearest object.
 * If more than one nearest objects exist, the smallest angle will be returned.
 * If no object is near enough, an angle of 90 degrees will be returned.
 *
 * @param int nMaxDistance: maximum distance of an object to be "near", defaults to 100cm
 * @return int nAngle: angle to nearest object
 *
 */
int getDirectionOfNearestObject(int nMaxDistance = 100) {
  int nMinAngle = 20;
  int nMaxAngle = 160;
  int nNearestObjectAngle = 90; // default
  int nNearestObjectDistance = 999;
  int nDistance = 0;

  for (int nAngle = nMinAngle; nAngle < nMaxAngle; nAngle += 15) {
    startServo1(nAngle);
    delay(DETACH_DELAY);
    stopServo1();
    nDistance = getDistance();

    if (nDistance < nNearestObjectDistance) {
      nNearestObjectDistance = nDistance;
      nNearestObjectAngle = nAngle;
    }
  }

  return nNearestObjectAngle;
}


void doBattle() {
  int nDistance = getDistance(); // ultrasonic.distanceRead(CM);
  int nAngleOfOpponent = 90;
  int nSpeed = 0;

  if (DEBUG) {
    // Serial.print("Distance to object: ");
    Serial.println(nDistance);
  }

  showDistance(90, nDistance);

  // set battle state

  // Entfernung zum Ziel < 30cm? Dann Attacke
  // sonst: links und rechts gucken und Ziel suchen.
  // In Richtung auf das nächste Ziel drehen
  // langsame Fahrt voraus

  // nBattleState:
  // 0 - lauern, Gegner suchen, Robat auf Gegner ausrichten
  // 1 - Angriff geradeaus
  // 2 - Angriff mit Schlenker links
  // 3 - Angriff mit Schlenker rechts
  // 4 - Zurückweichen
  // 5 - Suchfahrt geradeaus starten
  // 6 - Stop
  // 7 - Suchfahrt geradeaus

  switch (nBattleState) {
    case 0:
      // measure distance in different directions and turn to opponent
      nAngleOfOpponent = getDirectionOfNearestObject();
      nDistance = getDistanceDir(nAngleOfOpponent);

      // turn servo back to 90 degrees
      startServo1(90);
      delay(DETACH_DELAY);
      stopServo1();

      if (nAngleOfOpponent < 90) {
        // turn left
        turnRobot(LEFT, 4 * (90 - nAngleOfOpponent));
      }
      else if (nAngleOfOpponent > 90) {
        // turn right
        turnRobot(RIGHT, 4 * (nAngleOfOpponent - 90));
      }
      else {
        // opponent is right ahead, don't turn
      }

      if (nDistance < 20) {
        // Attacke
        nAttackCounter++;
        // TODO: different actions depending on number of attacks
        nBattleState = 1;
      }
      else {
        // Suchfahrt geradeaus
        nBattleState = 5;
      }
      break;
    case 1:
      nSpeed = 180; // full speed ahead
      startMotor(MOTOR_A, FORWARD, nSpeed);
      startMotor(MOTOR_B, FORWARD, nSpeed);
      delay(1000);
      nBattleState = 4; // zurücksetzen
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      nSpeed = 100;
      startMotor(MOTOR_A, BACKWARD, nSpeed);
      startMotor(MOTOR_B, BACKWARD, nSpeed);
      delay(800);
      stopMotor(MOTOR_A);
      stopMotor(MOTOR_B);
      nBattleState = 0;
      break;
    case 5:
      // Suchfahrt geradeaus starten
      nSpeed = 80;
      startMotor(MOTOR_A, FORWARD, nSpeed);
      startMotor(MOTOR_B, FORWARD, nSpeed);

      // get current time
      timeOfLastAction = millis();

      nBattleState = 7;
      break;
    case 6:
      stopMotor(MOTOR_A);
      stopMotor(MOTOR_B);
      break;
    case 7:
      // Suchfahrt geradeaus
      nDistance = getDistanceDir(90);
      if (nDistance < 20) {
        nBattleState = 1;
      }

      // change state on timeout
      if (millis() - timeOfLastDistanceMeasurement > 1000) {
        // // do not stop motors, just change state to search for opponent
        // stop motors and change state to search for opponent
        stopMotor(MOTOR_A);
        stopMotor(MOTOR_B);
        nBattleState = 0;
      }
      break;
  }
}

// --- END FUNCTIONS DO_BATTLE ---


// --- BEGIN FUNCTIONS AVOID_OBSTACLES ---

// depends on MOTOR, SERVO and ULTRASONIC

void avoidObstacles() {
  /*
  // turn about 45 degrees
  turnRobot(LEFT, 200);
  delay(1000);
  turnRobot(RIGHT, 200);
  delay(1000);

  // turn about 120 degrees
  turnRobot(LEFT, 500);
  delay(1000);
  turnRobot(RIGHT, 500);
  delay(1000);

  // turn about 230 degrees
  turnRobot(LEFT, 1000);
  delay(1000);
  turnRobot(RIGHT, 1000);
  delay(1000);
  */

  int nDistance = getDistance(); // ultrasonic.distanceRead(CM);
  if (DEBUG) {
    // Serial.print("Distance to object: ");
    Serial.println(nDistance);
  }
  // timeOfLastDistanceMeasurement = millis();

  showDistance(90, nDistance);

  int nSpeed = 0;
  if (digitalRead(BUMPER_PIN) == LOW) {
    stopMotor(MOTOR_A);
    stopMotor(MOTOR_B);
    delay(250);
    startMotor(MOTOR_A, BACKWARD, 128);
    startMotor(MOTOR_B, BACKWARD, 128);
    delay(750);
    stopMotor(MOTOR_A);
    delay(250);
    stopMotor(MOTOR_B);
  }
  else if (nDistance > 10) {
    // forward
    nSpeed = map(nDistance, 15, 150, 80, 128);
    startMotor(MOTOR_A, FORWARD, nSpeed);
    startMotor(MOTOR_B, FORWARD, nSpeed);
  }
  else {
    stopMotor(MOTOR_A);
    stopMotor(MOTOR_B);

    // make annoyed noise
    // tweet(TONE_PIN, 1);
    TimerFreeTone(TONE_PIN, 440, 80);
    TimerFreeTone(TONE_PIN, 0, 80);
    TimerFreeTone(TONE_PIN, 220, 200);

    // measure distance in different directions and turn to free direction
    startServo1(60);
    delay(DETACH_DELAY);
    stopServo1();
    nDistance = getDistance();

    if (nDistance > 15) {
      // turn left
      turnRobot(LEFT, 200);
    }
    else {
      startServo1(120);
      delay(DETACH_DELAY);
      stopServo1();
      nDistance = getDistance();

      if (nDistance > 15) {
        // turn right
        turnRobot(RIGHT, 200);
      }
      else {
        // turn around
        turnRobot(RIGHT, 1000);
      }

    }

    // set servo to middle position
    startServo1(90);
    delay(DETACH_DELAY);
    stopServo1();
  }

}

// --- END FUNCTIONS AVOID_OBSTACLES ---


// --- BEGIN FUNCTIONS MANUAL_CONTROL ---

// depends on MOTOR and JOYSTICK

void manualControl() {

  // Read the Joystick X and Y positions

  joyposVert = analogRead(joyVert);
  joyposHorz = analogRead(joyHorz);

/*
Serial.print(joyposVert);
Serial.print(",");
Serial.println(joyposHorz);
delay(500);
*/

  // Determine if this is a forward or backward motion
  // Do this by reading the Verticle Value
  // Apply results to MotorSpeed and to Direction

  if (joyposVert < 460)
  {
    // This is Backward

    // Set Motor A backward

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);

    // Set Motor B backward

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    //Determine Motor Speeds

    // As we are going backwards we need to reverse readings

    joyposVert = joyposVert - 460; // This produces a negative number
    joyposVert = joyposVert * -1;  // Make the number positive

    MotorSpeed1 = map(joyposVert, 0, 460, 0, MAX_SPEED);
    MotorSpeed2 = map(joyposVert, 0, 460, 0, MAX_SPEED);

  }
  else if (joyposVert > 564)
  {
    // This is Forward

    // Set Motor A forward

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);

    // Set Motor B forward

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    //Determine Motor Speeds

    MotorSpeed1 = map(joyposVert, 564, 1023, 0, MAX_SPEED);
    MotorSpeed2 = map(joyposVert, 564, 1023, 0, MAX_SPEED);

  }
  else
  {
    // This is Stopped

    MotorSpeed1 = 0;
    MotorSpeed2 = 0;

  }

  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor

  if (joyposHorz < 460)
  {
    // Move Left

    // As we are going left we need to reverse readings

    joyposHorz = joyposHorz - 460; // This produces a negative number
    joyposHorz = joyposHorz * -1;  // Make the number positive

    // Map the number to a value of 255 maximum

    joyposHorz = map(joyposHorz, 0, 460, 0, MAX_SPEED);


    MotorSpeed1 = MotorSpeed1 - joyposHorz;
    MotorSpeed2 = MotorSpeed2 + joyposHorz;

    // Don't exceed range of 0-255 for motor speeds

    if (MotorSpeed1 < 0)MotorSpeed1 = 0;
    if (MotorSpeed2 > MAX_SPEED)MotorSpeed2 = MAX_SPEED;

  }
  else if (joyposHorz > 564)
  {
    // Move Right

    // Map the number to a value of 255 maximum

    joyposHorz = map(joyposHorz, 564, 1023, 0, MAX_SPEED);


    MotorSpeed1 = MotorSpeed1 + joyposHorz;
    MotorSpeed2 = MotorSpeed2 - joyposHorz;

    // Don't exceed range of 0-255 for motor speeds

    if (MotorSpeed1 > MAX_SPEED)MotorSpeed1 = MAX_SPEED;
    if (MotorSpeed2 < 0)MotorSpeed2 = 0;

  }


  // Adjust to prevent "buzzing" at very low speed

  if (MotorSpeed1 < 8)MotorSpeed1 = 0;
  if (MotorSpeed2 < 8)MotorSpeed2 = 0;

  // Set the motor speeds

  analogWrite(enA, MotorSpeed1);
  analogWrite(enB, MotorSpeed2);

  if (true == DEBUG) {
    /*
    Serial.print("MotorSpeed1");
    Serial.println(MotorSpeed1);
    Serial.print("MotorSpeed2");
    Serial.println(MotorSpeed2);
    */
  }
}

// --- END FUNCTIONS MANUAL_CONTROL ---


void setup() {

// --- BEGIN SETUP GENERAL ---

  // send print statements at 9600 baud
  Serial.begin(115200);

// --- END SETUP GENERAL ---


// --- BEGIN SETUP BUMPER ---

  pinMode(BUMPER_PIN, INPUT_PULLUP);

// --- END SETUP BUMPER ---


// --- BEGIN SETUP SERVO ---
  pinMode(SERVO_2_PIN, OUTPUT);
  pinMode(SERVO_1_PIN, OUTPUT);

  // check servos
  startServo1(90);
  delay(500);
  stopServo1();
  startServo1(45);
  delay(500);
  stopServo1();
  startServo1(135);
  delay(500);
  stopServo1();

  delay(500);
  startServo1(90);
  delay(500);
  stopServo1();

  targetPositionServo1 = 90;
  actualPositionServo1 = 90;

  /*
  startServo2(90);
  delay(500);
  stopServo2();
  startServo2(45);
  delay(500);
  stopServo2();
  startServo2(135);
  delay(500);
  stopServo2();
  targetPositionServo2 = 0;
  actualPositionServo2 = 0;
  */

  /*
  int val = 0;

  servo1.attach(SERVO_1_PIN);
  bAttachedServo1 = true;
  val = 0;
  servo1.write(SERVO_1_MAX + (SERVO_1_MIN - SERVO_1_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  val = 90;
  servo1.write(SERVO_1_MAX + (SERVO_1_MIN - SERVO_1_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  val = 180;
  servo1.write(SERVO_1_MAX + (SERVO_1_MIN - SERVO_1_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  servo1.detach();
  bAttachedServo1 = false;
  */

// --- END SETUP SERVO ---



// --- BEGIN SETUP BUZZER ---

  for (int thisNote = 0; thisNote < melody_length; thisNote++) { // Loop through the notes in the array.
    TimerFreeTone(TONE_PIN, melody[thisNote], duration[thisNote]); // Play thisNote for duration.
    delay(50); // Short delay between notes.
  }

// --- END SETUP BUZZER ---


// --- BEGIN SETUP MOTOR ---

  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(stby, OUTPUT);

  // enable motor driver
  digitalWrite(stby, HIGH);

  // Start with motors disabled and direction forward

  // Motor A
  digitalWrite(enA, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Motor B
  digitalWrite(enB, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

// --- END SETUP MOTOR ---


// --- BEGIN SETUP AUTOMANUAL ---

  int nDistance = getDistance(); // ultrasonic.distanceRead(CM);

  if (nDistance < 10) {
    nMode = MANUAL;
  }
  else if (nDistance < 30) {
    nMode = AUTONOMOUS;
  }
  else {
    nMode = BATTLE;
  }

//if (digitalRead(BUMPER_PIN) == LOW) {
//  nMode = MANUAL;
//}

// --- BEGIN SETUP AUTOMANUAL ---


// --- BEGIN SETUP WS2811 ---

  pinMode(WS2811_PIN, OUTPUT);

  // initialize the LEDS
  FastLED.addLeds<WS2811, WS2811_PIN, RGB>(leds, NUM_LEDS);

  // set them all to be off
  //fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Headlights and backlights
  leds[0] = CRGB::Aqua;
  leds[1] = CRGB::Aqua;
  leds[2] = CRGB::White;
  leds[3] = CRGB::White;
  leds[4] = CRGB::Green;
  leds[5] = CRGB::Green;
  leds[6] = CRGB::White;
  leds[7] = CRGB::Red;
  leds[8] = CRGB::Blue;
  leds[9] = CRGB::Blue;
  leds[10] = CRGB::White;
  leds[11] = CRGB::White;

  FastLED.setBrightness(50);
  FastLED.show();

  delay(500);
  for (int i=0; i<NUM_LEDS; i++) {
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(50);
  }

  switch (nMode) {
    case AUTONOMOUS:
      leds[1] = CRGB::Red;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Black;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Red;
      FastLED.show();
      break;
    case BATTLE:
      leds[1] = CRGB::Green;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Black;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Green;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Black;
      FastLED.show();
      delay(500);
      leds[1] = CRGB::Green;
      FastLED.show();
      break;
    case MANUAL:
    default:
      leds[1] = CRGB::Blue;
      FastLED.show();
  }

// --- END SETUP WS2811 ---


// --- BEGIN SETUP JOYSTICK ---

  pinMode(joyVert, INPUT);
  pinMode(joyHorz, INPUT);

// --- END SETUP JOYSTICK ---

}

void loop() {

// --- BEGIN LOOP MOTOR ---

  switch (nMode) {
    case AUTONOMOUS:
      avoidObstacles();
      break;
    case BATTLE:
      doBattle();
      break;
    case MANUAL:
    default:
      manualControl();
  }

// --- END LOOP MOTOR ---

// --- BEGIN LOOP WS2811 ---
    /*
    // how bright to make this LED from 0 to 1, this is what makes them fade in and out
    // it calculates what percent we are completed with the letter, and makes it fade in from 0-50% and fade out from 50-100%
    // the formula can be visualized here: https://www.desmos.com/calculator/5qk8imeny4
    float brightness = 1-abs((2*(elapsed%millis_per_letter)/((float)millis_per_letter))-1);
    uint8_t value = 255 * brightness;
    */

    // get a rotation of colors, so that every NUM_COLORS lights, it loops
    // e.g. red, yellow, green, blue, red, yellow green blue
    // uint8_t hue = (letter_index%num_colors*255)/num_colors;

/*
    // set that LED to the color
    //leds[led] = CHSV(hue, 255, value);
    uint8_t hue = 0;
    uint8_t brightness = 255;
    for (int i=0; i < NUM_LEDS; i++) {

        //hue = (i % num_colors*255) / num_colors;
        hue = (random(0, NUM_LEDS) % num_colors*255) / num_colors;
        leds[i] = CHSV(hue, 255, brightness);
        FastLED.show();
    }
*/
    // set it to black so we don't have to remember the last LED we turned on
    // leds[led] = CRGB::Black;
// --- END LOOP WS2811 ---

// --- BEGIN LOOP SERVO ---
  // moveServoBackForth();
// --- END LOOP SERVO ---

// --- BEGIN LOOP ULTRASONIC ---
/*
  if (bMeasureDistance && millis() - timeOfLastDistanceMeasurement > DISTANCE_DELAY) {

    // measuring causes a delay of 692 ms
    int nDistance = ultrasonic.distanceRead(CM);
    if (DEBUG) {
      Serial.print("Distance to object: ");
      Serial.println(nDistance);
    }
    timeOfLastDistanceMeasurement = millis();

    // set LEDs to show distance
    leds[6] = CRGB::Red; // green
    leds[7] = CRGB::Red; // green
    leds[8] = CRGB::Red; // green
    FastLED.show();

  }
*/
// --- END LOOP ULTRASONIC ---


}