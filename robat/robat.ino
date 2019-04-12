/**
 *
 * RoBat - Robot Construction Kit
 *
 * Michael Ibsen
 * ibsen@gmx.net
 *
 * 2019-04-11
 *
 * License: GNU GPLv3
 *
 *
 */

// --- BEGIN INIT GENERAL ---

#define DEBUG false

// --- END INIT GENERAL ---


// --- BEGIN INIT AUTOMANUAL ---

// RoBat verfügt über drei verschiedene Betriebsmodi: 
// - manuelle Steuerung (MANUAL)
// - Hindernisvermeidung (AUTONOMOUS)
// - Hebocon- / Kampf (BATTLE)

#define MANUAL 0
#define AUTONOMOUS 1
#define BATTLE 2

// Standard-Modus ist MANUAL
int nMode = MANUAL;

// Zustand des Endlichen Automaten

// Im BATTLE Modus durchläuft das Programm verschiedene Zustände.
// Hier initialisieren wir die Zustandsvariablen

int nBattleState = 1; 
int nAttackCounter = 0;

// --- END INIT AUTOMANUAL ---


// --- BEGIN INIT BUMPER ---

// Mittels eines Schalters auf Pin BUMPER_PIN kann RoBat Kollisionen erkennen.
// Der Joystick-Button ist mit Pin 2 verbunden. Wenn BUMPER_PIN ebenfalls
// auf 2 gesetzt wird, wird ein Druck auf den Joystick-Button als Kollision
// interpretiert.

#define BUMPER_PIN 2

// --- END INIT BUMPER ---


// --- BEGIN INIT BUZZER ---

// Wir verwenden die TimerFreeTone Library von Tim Eckel in der Version 1.5
// https://bitbucket.org/teckel12/arduino-timer-free-tone/downloads/

#include <TimerFreeTone.h>

// Der Buzzer wird an TONE_PIN angeschlossen. Das muss ein PWM Pin sein,
// beim Arduino Nano also Pins 3, 5, 6, 9, 10 oder 11. Die RoBat-Platine
// benutzt Pin 11

#define TONE_PIN 11 

// Eine Zuordnung von Notennamen zu Frequenzen macht die Definition 
// von Melodien leichter lesbar.
// Quelle: https://de.wikipedia.org/wiki/Frequenzen_der_gleichstufigen_Stimmung

#define NOTE_C3 131
#define NOTE_D3 147
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_G3 196
#define NOTE_A3 220
#define NOTE_H3 247

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_H4 494

// Notenlängen (in ms)
#define GANZE 1000
#define HALBE 500
#define VIERTEL 250
#define ACHTEL 125

#define NOTE_PAUSE 0

// Einstellen einer Melodie
// Das Array melody[] enthält die Notenwerte, das gleich große Array
// duration[] die zugehörigen Notenlängen.
// Die Länge der Melodie wird durch melody_length festgelegt.

#define melody_length 2
int melody[]   = { NOTE_C4, NOTE_A4 };
int duration[] = { VIERTEL, ACHTEL  };

/*

// Eine andere Melodie

#define melody_length 8
int melody[]   = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, NOTE_PAUSE, NOTE_H3, NOTE_C4 };
int duration[] = { VIERTEL, ACHTEL,  ACHTEL,  VIERTEL, VIERTEL, VIERTEL,    VIERTEL, VIERTEL };

*/

// --- END INIT BUZZER ---


// --- BEGIN INIT SERVO ---

// Die Servo Bibliothek von Michael Margolis ist standardmäßig installiert.
// Dieser Sketch setzt auf v1.1.3 der Servo Bibliothek auf.

#include <Servo.h>

// servos on A0 (D14) and A1 (D15)

// RoBat verfügt über zwei Servos. Servo 2 wird zum Drehen des 
// Ultraschallsensors verwendet. Servo 1 ist frei verwendbar und wird
// in diesem Sketch langsam hin- und her bewegt.

// Für jeden Servo wird die minimale und maximale Position angegeben.
// Die minimale Position sollte theoretisch 0, die maximale 180 Grad sein. 
// In der Praxis werden diese Werte oft nicht erreicht. Sie müssen 
// individuell bestimmt werden. 

#define SERVO_1_PIN 13
#define SERVO_1_MIN 10   // minimale Position 0-180
#define SERVO_1_MAX 170  // maximale Position 0-180

#define SERVO_2_PIN 12
#define SERVO_2_MIN 25   // minimale Position 0-180
#define SERVO_2_MAX 180  // maximale Position 0-180

// Der Servo benötigt eine gewisse Zeit für die Drehung. DETACH_DELAY gibt
// diese Zeit in ms an.

#define DETACH_DELAY_SERVO_1 300 
#define DETACH_DELAY_SERVO_2 150 

// Servo Objekte erstellen

Servo servo1;
Servo servo2;

bool bAttachedServo1 = false;
bool bAttachedServo2 = false;

// Das Bewegen eines Servos dauert einige Zeit. Der Programmablauf soll
// nicht durch das Warten auf das Ende der Bewegung ausgebremst werden.
// Daher führen wir die Bewegung in kleinen Teilen aus. Wir merken uns 
// die Systemzeit bei Beginn der Bewegung und jeweils bei der Ausführung
// einer Teilbewegung. In der Loop prüfen wir, ob der Zeitpunkt für die
// nächste Bewegung gekommen ist und führen die nächste Teilbewegung 
// (nur) dann aus.

unsigned long timeOfLastChangeServo1 = 0;
unsigned long timeOfLastChangeServo2 = 0;

// Für die Bewegung erhält ein Servo eine Zielposition. Während der
// Teilbewegungen besitzt er jeweils eine aktuelle Position. Durch 
// Vergleich der beiden kann entschieden werden, ob eine weitere 
// Bewegung notwendig ist.

int actualPositionServo1 = 0;
int targetPositionServo1 = 0;
int actualPositionServo2 = 0;
int targetPositionServo2 = 0;

// --- END INIT SERVO ---


// --- BEGIN INIT ULTRASONIC ---

// Zur Entfernungsmessung wird ein Ultraschallsensor HC-SR04 eingesetzt.
// Der Messbereich liegt theoretisch zwischen 1cm und 400cm, in der Praxis
// sind Werte bis 200cm realistisch. 

// Wir verwenden die Ultrasonic Library von Erick Simões in der Version 3.0.0
// https://github.com/ErickSimoes/Ultrasonic

// Hinweis:
// Beim Compilieren kann es zu einer Fehlermeldung kommen:
// "as: unrecognized option '-mmcu=avr5'"
// Der Fehler lässt sich durch Verwendung der Version 1.6.9 des 
// Boardverwalters umgehen. 

#include <Ultrasonic.h>

// Pins für den Ultraschallsensor HC-SR04

#define TRIGGER_PIN  15  // Trigger Signal an den Ultraschallsensor
#define ECHO_PIN     14  // Echo Antwort vom Ultraschallsensor
#define MAX_DISTANCE 100 // Begrenzung der max. Distanz auf 100cm

unsigned long timeOfLastDistanceMeasurement = millis();

// Ultrasonic Objekt erzeugen

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
// ultrasonic.setTimeout(10000UL); // max. 40000UL

// --- END INIT ULTRASONIC ---


// --- BEGIN INIT WS2811 ---

// Für die Ansteuerung der LEDs (P9812 bzw. WS8212) verwenden wir die 
// FastLED Library von Daniel Garcia in der Version 3.2.6
// https://github.com/FastLED/FastLED

#include <FastLED.h>

// Anzahl der LEDs. RoBat verwendet standardmäßig nur zwei, aber mit 
// dieser Angabe können bis zu 12 hintereinandergeschaltet werden.
#define NUM_LEDS 12

// Array zur Speicherung der Farben aller LEDs
CRGB leds[NUM_LEDS];

// Die LEDs werden auf der RoBat Platine über Pin 10 angesteuert
#define WS2811_PIN 10

// --- END INIT WS2811 ---


// --- BEGIN INIT MOTOR ---

// Auf dem Motortreiberboard sind die Eingänge für Motor A mit in1 und in2
// markiert und die Eingänge für Motor B mit in3 und in4. 

// Motor A
#define MOTOR_A_ENABLE_PIN 6
#define MOTOR_A_IN1_PIN 7
#define MOTOR_A_IN2_PIN 8

// Motor B
#define MOTOR_B_ENABLE_PIN 3
#define MOTOR_B_IN3_PIN 5
#define MOTOR_B_IN4_PIN 4

#define MOTOR_STANDBY_PIN 9


// Geschwindigkeiten der beiden Motoren 
int nMotorSpeed1 = 0;
int nMotorSpeed2 = 0;

// Maximale Geschwindigkeit der beiden Motoren
// MAX_SPEED 0..255
#define MAX_SPEED 196

// Motor A / B (linker Motor / rechter Motor)
#define MOTOR_A 0
#define MOTOR_B 1

// Richtungen vorwärts und rückwärts eines Motors
#define FORWARD 0
#define BACKWARD 1

// Richtung, in die der Roboter sich dreht
#define LEFT 0
#define RIGHT 1

// --- END INIT MOTOR ---


// --- BEGIN INIT JOYSTICK ---

// Die Stellung des Joysticks wird über die Pins A6 und A7 eingelesen.
#define JOY_VERTICAL_PIN A6
#define JOY_HORIZONTAL_PIN A7

// Joystickposition (0..1023). Die Mittelstellung hat den Wert 512.
int nJoyPosV = 512;
int nJoyPosH = 512;

// --- END INIT JOYSTICK ---


// --- BEGIN FUNCTIONS ULTRASONIC ---

/**
 * Ermittelt die Distanz vom Roboter zum nächsten Hindernis mit dem
 * Ultraschallsensor. 
 * Die Entfernung wird dabei mehrfach gemessen, um Fehler zu minimieren.
 * Negative Entfernungen werden ignoriert (Messfehler).
 * 
 * Gibt den Durchschnittswert der Messungen zurück.
 *
 * @param int nMeasurements: Anzahl der Messungen, Default 3
 * @return int nDistanceAvg: Entfernung zum nächsten Objekt
 *
 */
int getDistance(int nMeasurements = 3) {
  int nDistanceAvg = 0;

  int nDistance = 0;
  int nDistanceSum = 0;
  int nValidValues = 0; // Anzahl gültiger (positiver) Messwerte

  for (int i = 0; i <= nMeasurements; i++) {
    nDistance = ultrasonic.read();
    
    // Hinweis: in früheren Versionen der Ultrasonic Bibliothek wurde
    // distanceRead() verwendet:
    // nDistance = ultrasonic.distanceRead(CM);

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


// --- BEGIN FUNCTIONS LED ---

/**
 * Zeigt die Entfernung zum nächsten Objekt mit den LEDs 0 und 1 durch
 * unterschiedliche Farben an.
 * 
 * 150cm - oo: schwarz 
 *  51cm - 150cm: grün
 *  31cm -  50cm: blau
 *  16cm -  30cm: türkis
 *   0cm -  15cm: rot
 *
 * @param int nDistance distance to object in this direction
 *
 */
void showDistance(int nDistance) {

    CRGB color = CRGB::Black;
    if (nDistance > 150) {
      color = CRGB::Black;
    }
    else if ((nDistance <= 150) && (nDistance > 50)) {
      color = CRGB::Red; // wird grün angezeigt
    }
    else if ((nDistance <= 50) && (nDistance > 30)) {
      color = CRGB::Blue;
    }
    else if ((nDistance <= 30) && (nDistance > 15)) {
      color = CRGB::DarkTurquoise;
    }
    else if (nDistance > 0) {
      color = CRGB::Green; // wird rot angezeigt
    }
    else {
      color = CRGB::Black;
    }

    // HINT: Diese Funktion ist zeitkritisch. Wenn mehr als 2 LEDs 
    // (bzw. deren Array-Elemente) gesetzt werden, laufen die Motoren 
    // nicht mehr. 

    leds[0] = color;
    leds[1] = color;
    
    FastLED.show();
}

// --- END FUNCTIONS LED ---


// --- BEGIN FUNCTIONS SERVO ---

/**
 * Stellt die Zielposition für Servo 1 ein und schaltet den Servomotor ein.
 * 
 */
void startServo1(int targetPos) {
    targetPositionServo1 = targetPos;

    // starte Servo
    servo1.attach(SERVO_1_PIN);
    bAttachedServo1 = true; // merken, dass der Servo eingeschaltet ist

    // Setzt die Servo-Position (0..180)
    // Dabei werden die minimale und maximale Position berücksichtigt.
    int targetPosCorrected = map(targetPositionServo1, 0, 180, SERVO_1_MIN, SERVO_1_MAX);
    servo1.write(targetPosCorrected);
        
    if (DEBUG) {
      Serial.print("actualPositionServo1: ");
      Serial.println(actualPositionServo1);
      Serial.print("targetPositionServo1: ");
      Serial.println(targetPositionServo1);
    }

    actualPositionServo1 = -1;
}


/**
 * Stellt die Zielposition für Servo 2 ein und schaltet den Servomotor ein.
 * 
 */
void startServo2(int targetPos) {
    targetPositionServo2 = targetPos;

    // starte Servo
    servo2.attach(SERVO_2_PIN);
    bAttachedServo2 = true;

    // Setzt die Servo-Position (0..180)
    // Dabei werden die minimale und maximale Position berücksichtigt.
    int targetPosCorrected = map(targetPositionServo2, 0, 180, SERVO_2_MIN, SERVO_2_MAX);
    servo2.write(targetPosCorrected);

    if (DEBUG) {
      Serial.print("actualPositionServo2: ");
      Serial.println(actualPositionServo2);
      Serial.print("targetPositionServo2: ");
      Serial.println(targetPositionServo2);
      Serial.print("targetPos (corrected): ");
      Serial.println(targetPosCorrected);
    }

    actualPositionServo2 = -1;
}


/**
 * Hält Servo 1 an.
 * 
 */
void stopServo1() {
   servo1.detach();
   bAttachedServo1 = false;
   actualPositionServo1 = targetPositionServo1;

   if (DEBUG) {
       Serial.println("stopping servo 1");
   }
}


/**
 * Hält Servo 2 an.
 * 
 */
void stopServo2() {
   servo2.detach();
   bAttachedServo2 = false;
   actualPositionServo2 = targetPositionServo2;

   if (DEBUG) {
       Serial.println("stopping servo 2");
   }
}


/**
 * Bewegt Servo 1 hin und her. Dabei werden nacheinander mehrere 
 * Positionen angesteuert.
 * 
 */
void moveServoBackForth() {

    unsigned long timeNow = millis();

    // Positionen sind 60, 90 und 130 Grad

    if (0 == actualPositionServo1) {
        targetPositionServo1 = 60;
        startServo1(targetPositionServo1);
        timeOfLastChangeServo1 = timeNow;
    }
    else if (60 == actualPositionServo1) {
        targetPositionServo1 = 90;
        startServo1(targetPositionServo1);
        timeOfLastChangeServo1 = timeNow;
    }
    else if (90 == actualPositionServo1) {
        targetPositionServo1 = 130;
        startServo1(targetPositionServo1);
        timeOfLastChangeServo1 = timeNow;
    }
    else if (130 == actualPositionServo1) {
        targetPositionServo1 = 60;
        startServo1(targetPositionServo1);
        timeOfLastChangeServo1 = timeNow;
    }

    // Servo anhalten, wenn die Position erreicht ist bzw. die Zeit für die 
    // Erreichung der Position abgelaufen ist.
    if (bAttachedServo1 && (timeNow - timeOfLastChangeServo1) > DETACH_DELAY_SERVO_1) {
        if (DEBUG) {
            Serial.print("servo time:");
            Serial.println(timeNow - timeOfLastChangeServo1);
        }

        stopServo1();

        actualPositionServo1 = targetPositionServo1;
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

  if (DEBUG) {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print("Starting motor ");
    Serial.print(nMotor);
    Serial.print(", dir: ");
    Serial.print(nDir);
    Serial.print(", speed: ");
    Serial.println(nSpeed);
  }
  
  // limit speed to 0 .. MAX_SPEED
  nSpeed = max(0, min(nSpeed, MAX_SPEED));

  if (MOTOR_A == nMotor) {
    if (FORWARD == nDir) {
      // motor A forward
      // Set Motor A forward
      digitalWrite(MOTOR_A_IN1_PIN, HIGH);
      digitalWrite(MOTOR_A_IN2_PIN, LOW);
    }
    else {
      // motor A backward
      // Set Motor A backward
      digitalWrite(MOTOR_A_IN1_PIN, LOW);
      digitalWrite(MOTOR_A_IN2_PIN, HIGH);
    }
    analogWrite(MOTOR_A_ENABLE_PIN, nSpeed);
  }
  else {
    if (FORWARD == nDir) {
      // motor B forward
      // Set Motor B forward
      digitalWrite(MOTOR_B_IN3_PIN, HIGH);
      digitalWrite(MOTOR_B_IN4_PIN, LOW);
    }
    else {
      // motor B backward
      // Set Motor B backward
      digitalWrite(MOTOR_B_IN3_PIN, LOW);
      digitalWrite(MOTOR_B_IN4_PIN, HIGH);
    }
    analogWrite(MOTOR_B_ENABLE_PIN, nSpeed);
  }
}

/**
 * Start motor
 *
 * @param int nMotor: 0 - motor A, 1 - motor B
 */
void stopMotor(int nMotor) {

  if (DEBUG) {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print("stopping motor ");
    Serial.println(nMotor);
  }
  
  if (MOTOR_A == nMotor) {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    analogWrite(MOTOR_A_ENABLE_PIN, 0); // set high to switch motor off, 0 for short brake
  }
  else {
    digitalWrite(MOTOR_B_IN3_PIN, LOW);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);
    analogWrite(MOTOR_B_ENABLE_PIN, 0); // set high to switch motor off, 0 for short brake
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

  startServo2(nAngle);
  delay(DETACH_DELAY_SERVO_2);
  stopServo2();
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
  int nMinAngle = 30; // 20
  int nMaxAngle = 150; // 160
  int nNearestObjectAngle = 90; // default
  int nNearestObjectDistance = 999;
  int nDistance = 0;

  for (int nAngle = nMinAngle; nAngle < nMaxAngle; nAngle += 30) { // 15
    startServo2(nAngle);
    delay(DETACH_DELAY_SERVO_2);
    stopServo2();
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
    Serial.print("distance to object: ");
    Serial.print(nDistance);
    Serial.print(", ");
    Serial.print("battle state: ");
    Serial.println(nBattleState);
    
  }

  // Entfernung mit LEDs anzeigen
  showDistance(nDistance);

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
      startServo2(90);
      delay(DETACH_DELAY_SERVO_2);
      stopServo2();

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
        // nAttackCounter++;
        // TODO: different actions depending on number of attacks
        nBattleState = 1;
      }
      else {
        // Suchfahrt geradeaus
        nBattleState = 5;
      }
      break;
    case 1:
      nAttackCounter++; // Anzahl durchgeführter Angriffe
      
      nSpeed = 180; // full speed ahead
      startMotor(MOTOR_A, FORWARD, nSpeed);
      startMotor(MOTOR_B, FORWARD, nSpeed);
      delay(1000);
      
      stopMotor(MOTOR_A);
      stopMotor(MOTOR_B);

      
      if (1 == nAttackCounter) {
        // beim ersten Angriff nur vorwärts fahren
        nBattleState = 0; // lauern
      }
      else {
        nBattleState = 4; // rückwärts fahren
      }
      
      //nBattleState = 4; // rückwärts fahren
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

  showDistance(nDistance);

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
    startServo2(60);
    delay(DETACH_DELAY_SERVO_2);
    stopServo2();
    nDistance = getDistance();

    if (nDistance > 15) {
      // turn left
      turnRobot(LEFT, 200);
    }
    else {
      startServo2(120);
      delay(DETACH_DELAY_SERVO_2);
      stopServo2();
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
    startServo2(90);
    delay(DETACH_DELAY_SERVO_2);
    stopServo2();
  }

}

// --- END FUNCTIONS AVOID_OBSTACLES ---


// --- BEGIN FUNCTIONS MANUAL_CONTROL ---

// depends on MOTOR and JOYSTICK

void manualControl() {

  // Read the Joystick X and Y positions

  nJoyPosV = analogRead(JOY_VERTICAL_PIN);
  nJoyPosH = analogRead(JOY_HORIZONTAL_PIN);

/*
Serial.print(nJoyPosV);
Serial.print(",");
Serial.println(nJoyPosH);
delay(500);
*/

  // Determine if this is a forward or backward motion
  // Do this by reading the Verticle Value
  // Apply results to MotorSpeed and to Direction

  if (nJoyPosV < 460)
  {
    // This is Backward

    // Set Motor A backward

    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, HIGH);

    // Set Motor B backward

    digitalWrite(MOTOR_B_IN3_PIN, LOW);
    digitalWrite(MOTOR_B_IN4_PIN, HIGH);

    //Determine Motor Speeds

    // As we are going backwards we need to reverse readings

    nJoyPosV = nJoyPosV - 460; // This produces a negative number
    nJoyPosV = nJoyPosV * -1;  // Make the number positive

    nMotorSpeed1 = map(nJoyPosV, 0, 460, 0, MAX_SPEED);
    nMotorSpeed2 = map(nJoyPosV, 0, 460, 0, MAX_SPEED);

  }
  else if (nJoyPosV > 564)
  {
    // This is Forward

    // Set Motor A forward

    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    // Set Motor B forward

    digitalWrite(MOTOR_B_IN3_PIN, HIGH);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);

    //Determine Motor Speeds

    nMotorSpeed1 = map(nJoyPosV, 564, 1023, 0, MAX_SPEED);
    nMotorSpeed2 = map(nJoyPosV, 564, 1023, 0, MAX_SPEED);

  }
  else
  {
    // This is Stopped

    nMotorSpeed1 = 0;
    nMotorSpeed2 = 0;

  }

  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor

  if (nJoyPosH < 460)
  {
    // Move Left

    // As we are going left we need to reverse readings

    nJoyPosH = nJoyPosH - 460; // This produces a negative number
    nJoyPosH = nJoyPosH * -1;  // Make the number positive

    // Map the number to a value of 255 maximum

    nJoyPosH = map(nJoyPosH, 0, 460, 0, MAX_SPEED);


    nMotorSpeed1 = nMotorSpeed1 - nJoyPosH;
    nMotorSpeed2 = nMotorSpeed2 + nJoyPosH;

    // Don't exceed range of 0-255 for motor speeds

    if (nMotorSpeed1 < 0)nMotorSpeed1 = 0;
    if (nMotorSpeed2 > MAX_SPEED)nMotorSpeed2 = MAX_SPEED;

  }
  else if (nJoyPosH > 564)
  {
    // Move Right

    // Map the number to a value of 255 maximum

    nJoyPosH = map(nJoyPosH, 564, 1023, 0, MAX_SPEED);


    nMotorSpeed1 = nMotorSpeed1 + nJoyPosH;
    nMotorSpeed2 = nMotorSpeed2 - nJoyPosH;

    // Don't exceed range of 0-255 for motor speeds

    if (nMotorSpeed1 > MAX_SPEED)nMotorSpeed1 = MAX_SPEED;
    if (nMotorSpeed2 < 0)nMotorSpeed2 = 0;

  }


  // Adjust to prevent "buzzing" at very low speed

  if (nMotorSpeed1 < 8)nMotorSpeed1 = 0;
  if (nMotorSpeed2 < 8)nMotorSpeed2 = 0;

  // Set the motor speeds

  analogWrite(MOTOR_A_ENABLE_PIN, nMotorSpeed1);
  analogWrite(MOTOR_B_ENABLE_PIN, nMotorSpeed2);

  if (true == DEBUG) {
    /*
    Serial.print("nMotorSpeed1");
    Serial.println(nMotorSpeed1);
    Serial.print("nMotorSpeed2");
    Serial.println(nMotorSpeed2);
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
  pinMode(SERVO_1_PIN, OUTPUT);
  pinMode(SERVO_2_PIN, OUTPUT);

  // check servos
  startServo2(90);
  startServo1(90);
  delay(500);
  stopServo2();
  startServo2(45);
  stopServo1();
  startServo1(45);
  delay(500);
  stopServo2();
  startServo2(135);
  stopServo1();
  startServo1(135);
  delay(500);
  stopServo2();
  stopServo1();

  delay(500);
  startServo2(90);
  startServo1(90);
  delay(500);
  stopServo2();
  stopServo1();

  targetPositionServo2 = 90;
  actualPositionServo2 = 90;
  targetPositionServo1 = 90;
  actualPositionServo1 = 90;

  /*
  startServo1(90);
  delay(500);
  stopServo1();
  startServo1(45);
  delay(500);
  stopServo1();
  startServo1(135);
  delay(500);
  stopServo1();
  targetPositionServo1 = 0;
  actualPositionServo1 = 0;
  */

  /*
  int val = 0;

  servo2.attach(SERVO_2_PIN);
  bAttachedServo2 = true;
  val = 0;
  servo2.write(SERVO_2_MAX + (SERVO_2_MIN - SERVO_2_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  val = 90;
  servo2.write(SERVO_2_MAX + (SERVO_2_MIN - SERVO_2_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  val = 180;
  servo2.write(SERVO_2_MAX + (SERVO_2_MIN - SERVO_2_MAX) / 100 * val); // sets the servo position 0-180
  delay(500);
  servo2.detach();
  bAttachedServo2 = false;
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
  pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_B_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN3_PIN, OUTPUT);
  pinMode(MOTOR_B_IN4_PIN, OUTPUT);

  pinMode(MOTOR_STANDBY_PIN, OUTPUT);

  // enable motor driver
  digitalWrite(MOTOR_STANDBY_PIN, HIGH);

  // Start with motors disabled and direction forward

  // Motor A
  digitalWrite(MOTOR_A_ENABLE_PIN, LOW);
  digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  digitalWrite(MOTOR_A_IN2_PIN, LOW);

  // Motor B
  digitalWrite(MOTOR_B_ENABLE_PIN, LOW);
  digitalWrite(MOTOR_B_IN3_PIN, HIGH);
  digitalWrite(MOTOR_B_IN4_PIN, LOW);

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
      // set LED color according to joystick position
      int nJoyHPos = analogRead(JOY_HORIZONTAL_PIN);
      int nJoyColorValue = map(nJoyHPos, 0, 1023, 0, 255);
      
      //leds[0] = CRGB::Blue;
      //leds[1] = CRGB::Blue;

      //Example 1: set color from red, green, and blue components individually
      leds[0].red = nJoyColorValue;
      leds[0].green = 255 - nJoyColorValue;
      leds[0].blue =  150;
      
      //Example 2: set color from red, green, and blue components all at once
      leds[1] = CRGB( nJoyColorValue, 255 - nJoyColorValue, 150);
      FastLED.show();
  }

// --- END SETUP WS2811 ---


// --- BEGIN SETUP JOYSTICK ---
  // don't do this as it somehow disturbs the timer for PWM 
  //pinMode(joyVert, INPUT);
  //pinMode(joyHorz, INPUT);

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
  moveServoBackForth();
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
