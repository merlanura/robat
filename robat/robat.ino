/**
 * Steuersoftware für den Roboterbausatz RoBat
 * Control software for the RoBat robot kit
 *
 * https://github.com/merlanura/robat
 * https://robot-workshop.de/
 *
 * Michael Ibsen
 * ibsen@gmx.net
 *
 * 2021-12-10
 *
 * License: GNU GPLv3
 * http://www.gnu.de/documents/gpl.de.html
 *
 * Dieses Programm ist freie Software. Sie können es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation
 * veröffentlicht, weitergeben und/oder modifizieren, entweder gemäß
 * Version 3 der Lizenz oder (nach Ihrer Option) jeder späteren Version.
 *
 * Die Veröffentlichung dieses Programms erfolgt in der Hoffnung, daß es
 * Ihnen von Nutzen sein wird, aber OHNE IRGENDEINE GARANTIE, sogar ohne
 * die implizite Garantie der MARKTREIFE oder der VERWENDBARKEIT FÜR
 * EINEN BESTIMMTEN ZWECK. Details finden Sie in der GNU General Public
 * License.
 *
 * Sie sollten ein Exemplar der GNU General Public License zusammen mit
 * diesem Programm erhalten haben. Falls nicht, siehe <http://www.gnu.org/licenses/>.
 *
 */

// --- BEGIN INIT GENERAL ---

#define DEBUG false

// Wenn ein OLED Display über i2c angeschlossen ist,
// kann hier HASOLED definiert werden, um Code für 
// das Display einzubinden. Anderenfalls wird HASOLED
// entfernt.
// If RoBat has an OLED display on i2c (pins A4 and A5),
// define HASOLED here to include code for the display. 
// Otherwise undefine HASOLED.
// #define HASOLED true
#undef HASOLED

// --- END INIT GENERAL ---


// --- BEGIN INIT MODE ---

// RoBat verfügt über vier verschiedene Betriebsmodi:
// - manuelle Steuerung (MANUAL)
// - Hindernisvermeidung (AUTONOMOUS)
// - Hebocon / Kampf (BATTLE)
// - Steuerung durch externe Controller über serielle Schnittstelle (SERIAL)

// RoBat has four different modes of operation:
// - manual control (MANUAL)
// - obstacle avoidance (AUTONOMOUS)
// - Hebocon / battle (BATTLE)
// - external control via the serial interface (SERIAL)

#define MANUAL 0
#define AUTONOMOUS 1
#define BATTLE 2
#define SERIALCMD 3


// Standard-Modus ist MANUAL
// default mode is MANUAL
int nMode = MANUAL;

// Zustand des Endlichen Automaten

// Im BATTLE Modus durchläuft das Programm verschiedene Zustände.
// Hier initialisieren wir die Zustandsvariablen

// While in battle mode the program will cycle through different states.
// initialise the state variables

int nBattleState = 1;
int nAttackCounter = 0;

// --- END INIT MODE ---


// --- BEGIN INIT OLED ---

#ifdef HASOLED

// OLED Display

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4 // not used / nicht genutzt bei diesem Display
Adafruit_SSD1306 display(OLED_RESET);

#define DRAW_DELAY 118
#define D_NUM 47
#define SSD1306_128_64 true

String displaylines[4] = {"Hello there!", "I am RoBat", "",  ""};

#endif

// --- END INIT OLED ---


// --- BEGIN INIT BUMPER ---

// Mittels Schaltern auf den Pins BUMPER1_PIN und BUMPER2_PIN kann RoBat
// Kollisionen erkennen. Die analogen Eingänge A2 und A3 werden dafür in setup()
// als digitale Eingänge konfiguriert.
// Der Joystick-Button ist mit Pin JOYSTICK_SWITCH_PIN verbunden. Bei Bedarf kann dieser Pin
// zusätzlich zur Kollisionserkennung eingesetzt werden.

// RoBat can detect collisions using switches on pins BUMPER1_PIN and BUMPER2_PIN.
// The analog input pins A2 and A3 are configured as digital inputs in setup() for
// this purpose. 
// The joystick button is connected to pin JOYSTICK_SWITCH_PIN. It can 
// also be used for collision detection if necessary. 

#define JOYSTICK_SWITCH_PIN 2
#define BUMPER1_PIN A2
#define BUMPER2_PIN A3

// --- END INIT BUMPER ---


// --- BEGIN INIT BUZZER ---

// Wir verwenden die TimerFreeTone Library von Tim Eckel in der Version 1.5
// https://bitbucket.org/teckel12/arduino-timer-free-tone/downloads/

// We are using the library TimerFreeTone v1.5 by Tim Eckel
// https://bitbucket.org/teckel12/arduino-timer-free-tone/downloads/

#include <TimerFreeTone.h>

// Der Buzzer wird an TONE_PIN angeschlossen. Das muss ein PWM Pin sein,
// beim Arduino Nano also Pins 3, 5, 6, 9, 10 oder 11. Die RoBat-Platine
// benutzt Pin 11

// The buzzer is connected to pin TONE_PIN. This has to be a PWM pin.
// The Arduino Nano offers PWM on pins 3, 5, 6, 9, 10 and 11. On the 
// RoBat circuit board the buzzer is connected to pin 11.

#define TONE_PIN 11

// Eine Zuordnung von Notennamen zu Frequenzen macht die Definition
// von Melodien leichter lesbar.
// Quelle: https://de.wikipedia.org/wiki/Frequenzen_der_gleichstufigen_Stimmung

// Defining names for notes makes it easier to encode melodies.
// Source: https://de.wikipedia.org/wiki/Frequenzen_der_gleichstufigen_Stimmung

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

#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_A5 880
#define NOTE_H5 988

// Notenlängen (in ms)
// duration of the notes (ms)
#define GANZE 1000
#define HALBE 500
#define VIERTEL 250
#define ACHTEL 125

#define NOTE_PAUSE 0

// Einstellen einer Melodie
// Das Array melody[] enthält die Notenwerte, das gleich große Array
// duration[] die zugehörigen Notenlängen.
// Die Länge der Melodie wird durch melody_length festgelegt.

// Melody
// The array melody[] contains the notes, the array duration[] must be of
// the same length and contains the duration for each of the notes in 
// melody[]
#define melody_length 2
int melody[]   = { NOTE_C4, NOTE_A4 };
int duration[] = { VIERTEL, ACHTEL  };

/*

// Eine andere Melodie
// another melody

#define melody_length 8
int melody[]   = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, NOTE_PAUSE, NOTE_H3, NOTE_C4 };
int duration[] = { VIERTEL, ACHTEL,  ACHTEL,  VIERTEL, VIERTEL, VIERTEL,    VIERTEL, VIERTEL };

*/

// --- END INIT BUZZER ---


// --- BEGIN INIT SERVO ---

// Die Servo Bibliothek von Michael Margolis ist standardmäßig installiert.
// Dieser Sketch setzt auf v1.1.3 der Servo Bibliothek auf.

// The servo library by Michael Margolis is part of the Arduino suite.
// This program was tested with version 1.1.3 of the library.
#include <Servo.h>

// servos on A0 (D14) and A1 (D15)

// RoBat verfügt über zwei Servos. Servo 2 wird zum Drehen des
// Ultraschallsensors verwendet. Servo 1 ist frei verwendbar und wird
// in diesem Sketch langsam hin- und her bewegt.

// Für jeden Servo wird die minimale und maximale Position angegeben.
// Die minimale Position sollte theoretisch 0, die maximale 180 Grad sein.
// In der Praxis werden diese Werte oft nicht erreicht. Sie müssen
// individuell bestimmt werden.

// RoBat supports two servos, although only one is normally used.
// Servo 2 is used to turn the ultrasonic distance sensor. Servo 1 can
// be used for other purposes. RoBat turns it back and forth.

// In theory, each servo can be set at any position from 0 to 180 degrees.
// In practice, the extreme positions sometimes cannot be reached. 
// The reachable positions as well as the pins for each servo are 
// defined below.

#define SERVO_1_PIN 13
#define SERVO_1_MIN 10   // minimale Position 0-180
#define SERVO_1_MAX 170  // maximale Position 0-180

#define SERVO_2_PIN 12
#define SERVO_2_MIN 10   // minimale Position 0-180
#define SERVO_2_MAX 170  // maximale Position 0-180

// Der Servo benötigt eine gewisse Zeit für die Drehung. DETACH_DELAY gibt
// diese Zeit in ms an.

// Turning the servo takes some time. DETACH_DELAY_SERVO_N defines the
// time (ms) before the servo is turned off again.

#define DETACH_DELAY_SERVO_1 300
#define DETACH_DELAY_SERVO_2 150

// Servo Objekte erstellen
// create servo objects

Servo servo1;
Servo servo2;

bool bAttachedServo1 = false;
bool bAttachedServo2 = false;

// Das Bewegen eines Servos dauert einige Zeit. Der Programmablauf soll
// nicht durch das Warten auf das Ende der Bewegung ausgebremst werden.
// Daher führen wir die Bewegung in kleinen Teilen aus. Wir merken uns
// die Systemzeit bei Beginn der Bewegung und jeweils bei der Ausführung
// einer Teilbewegung. In der loop() prüfen wir, ob der Zeitpunkt für die
// nächste Bewegung gekommen ist und führen die nächste Teilbewegung
// (nur) dann aus.

// It takes some time to set the servo to a position. The program should 
// not be delayed by controlling the servo. Therefore, the servo is 
// moved in tiny steps. The current system time will be stored at the 
// beginning of the movement and also at each step. In the loop() we 
// check if the time for the next movement is reached and move the servo 
// (only) if this is tha case.

unsigned long timeOfLastChangeServo1 = 0;
unsigned long timeOfLastChangeServo2 = 0;

// Für die Bewegung erhält ein Servo eine Zielposition. Während der
// Teilbewegungen besitzt er jeweils eine aktuelle Position. Durch
// Vergleich der beiden kann entschieden werden, ob eine weitere
// Bewegung notwendig ist.

// The servo has a target position and a current position while moving.
// By comparing the two values, the program decides if further
// movement of the servo is necessary.

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

// Like a bat, RoBat has an ultrasonic sensor for measuring the distance
// to near objects. That is actually where RoBats name comes from.
// The ultrasonic sensor is an HC-SR04 with a range between 1cm and 4m.
// In practice, distances up to 200cm are decently accurate.

// The program uses version 3.0.0 of Eric Simões' ultrasonic library.
// https://github.com/ErickSimoes/Ultrasonic

// Hint:
// An error may occur at compile time:
// "as: unrecognized option '-mmcu=avr5'"
// The error can be avoided by using version 1.6.9 of the Boards Manager
// in the Arduino IDE.

#include <Ultrasonic.h>

// Pins für den Ultraschallsensor HC-SR04
// pins used for the ultrasonic sensor HC-SR04

// Trigger Signal an den Ultraschallsensor
// trigger signal sent to the sensor
#define TRIGGER_PIN 15

// Echo Antwort vom Ultraschallsensor
// reply from the sensor
#define ECHO_PIN 14 

// Begrenzung der max. Distanz auf 200cm
// maximum distance which can be measured
#define MAX_DISTANCE 200 

unsigned long timeOfLastDistanceMeasurement = millis();

// Ultrasonic Objekt erzeugen
// create the sensor object

Ultrasonic ultrasonic(TRIGGER_PIN, ECHO_PIN);
// ultrasonic.setTimeout(10000UL); // max. 40000UL

// --- END INIT ULTRASONIC ---


// --- BEGIN INIT WS2811 ---

// Für die Ansteuerung der LEDs (P9812 bzw. WS8212) verwenden wir die
// FastLED Library von Daniel Garcia in der Version 3.2.6
// https://github.com/FastLED/FastLED

// We use the FastLED library by Daniel Garcia in version 3.2.6 to
// control the RGB LEDs (P9812 or WS8212)
// https://github.com/FastLED/FastLED

#include <FastLED.h>

// Anzahl der LEDs. RoBat verwendet standardmäßig nur zwei, aber mit
// dieser Angabe können bis zu 12 hintereinandergeschaltet werden.

// Number of LEDs connected. RoBat usually only has two, but up to 12 
// are possible.
#define NUM_LEDS 12

// Array zur Speicherung der Farben aller LEDs
// array to store the color for each LED
CRGB leds[NUM_LEDS];

// Die LEDs werden auf der RoBat Platine über Pin 10 angesteuert.
// The LEDs are connected to pin 10 of the Arduino on the circuit board. 
#define WS2811_PIN 10

// --- END INIT WS2811 ---


// --- BEGIN INIT MOTOR ---

// Auf dem Motortreiberboard sind die Eingänge für Motor A mit in1 und in2
// markiert und die Eingänge für Motor B mit in3 und in4.

// On the motor driver module the inputs for motor A are in1 and in2
// and in3 and in4 for motor B.

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
// speed for each motor
int nMotorSpeed1 = 0;
int nMotorSpeed2 = 0;

// Vorherige Motorgeschwindigkeiten
// previous speed of the motors
int nPrevMotorSpeed1 = 0;
int nPrevMotorSpeed2 = 0;

// Maximale Geschwindigkeit der beiden Motoren
// maximum speed of both motors
// nMaxSpeed 0..255
int nMaxSpeed = 255;

// Für die Motoren und ihre Laufrichtung werden Namen definiert, um die
// Ansteuerung zu erleichtern.

// We define names for the motors and their direction to make programming
// easier.

// Motor A / B (linker Motor / rechter Motor)
// motor A / B (left motor / right motor)
#define MOTOR_A 0
#define MOTOR_B 1

// Richtungen vorwärts und rückwärts eines Motors
// direction (forward / backward) for the motors
#define FORWARD 0
#define BACKWARD 1
#define STOP 2

// Richtung, in die der Roboter sich dreht
// direction (left / right) in which the robot turns
#define LEFT 0
#define RIGHT 1

// --- END INIT MOTOR ---


// --- BEGIN INIT JOYSTICK ---

// Die Stellung des Joysticks wird über die Pins A6 und A7 eingelesen.
// pins for the horizontal and vertical direction of the joystick
#define JOY_VERTICAL_PIN A6
#define JOY_HORIZONTAL_PIN A7

// Joystickposition (0..1023). Die Mittelstellung hat den Wert 512.
// position of the joystick. The neutral position is 512.
int nJoyPosV = 512;
int nJoyPosH = 512;

// Werte zwischen JOY_MIDDLE_MIN und JOY_MIDDLE_MAX werden als
// Mittelstellung interpretiert

// Values between JOY_MIDDLE_MIN and JOY_MIDDLE_MAX are interpreted as
// neutral position.
#define JOY_MIDDLE_MIN 472
#define JOY_MIDDLE_MAX 552

// --- END INIT JOYSTICK ---


// --- BEGIN INIT SERIAL COMMANDS ---

#define SERIALSPEED 38400

// String für eingehende serielle Zeichen
// buffer to store incoming serial commands
String strSerialInput = ""; 

// Markierung für das Ende eines Kommandos, wird in serialEvent() 
// bei Eintreffen von \n oder \r auf true gesetzt.

// Marker for the end of a serial command. Is set to true in serialEvent()
// when \n or \r characters are received
bool bSerialComplete = false; 

// Die Motoren werden nach einer Zeit ohne serielle Kommandos
// ausgeschaltet. Dadurch hält der Roboter an, wenn die serielle
// Verbindung abbricht.

// Stop motors if no serial commands are received for some time. This 
// sets the robot into a safe state if the serial connection is broken.
unsigned long nMotorTimeout = 400; // ms
unsigned long nLastCmd = millis();

// --- END INIT SERIAL COMMANDS ---


// --- BEGIN FUNCTIONS OLED ---

#ifdef HASOLED

/**
 * Zeigt den aktuellen Betriebsmodus auf dem 
 * OLED Display an.
 * 
 * Shows the current operating mode on the 
 * OLED display.
 * 
 */
void showMode() {
  // Display leeren
  // Clear old content from display
  display.clearDisplay();
  
  // Textstartposition einstellen
  // set text cursor position 
  display.setCursor(1, 0);

  // Betriebsmodus anzeigen
  // show operation mode
  switch (nMode) {
    case MANUAL:
      displaylines[3] = "mode: MANUAL";
      break;
    case AUTONOMOUS:
      displaylines[3] = "mode: AUTO";
      break;
    case BATTLE:
      displaylines[3] = "mode: BATTLE";
      break;
    case SERIALCMD:
      displaylines[3] = "mode: SERIAL";
      break;
    default:
      displaylines[3] = "mode: NONE";
  }

  // Text anzeigen / show welcome message
  for (int i=0; i<4; i++) {
    display.println(displaylines[i]);
  }
  display.display();

}


/**
 * Zeigt die Entfernung zum nächsten Objekt auf dem 
 * OLED Display an.
 * 
 * Shows the distance to the nearest object on the 
 * OLED display.
 * 
 */
void showDistanceOLED(int nDistance) {
  // Display leeren
  // Clear old content from display
  display.clearDisplay();
  
  // Textstartposition einstellen
  // set text cursor position 
  display.setCursor(1, 0);

  String strDistance = String(nDistance);
  displaylines[2] = String("dist: " + strDistance);

  // Text anzeigen / show welcome message
  for (int i=0; i<4; i++) {
    display.println(displaylines[i]);
  }
  display.display();

}

#endif

// --- END FUNCTIONS OLED ---


// --- BEGIN FUNCTIONS ULTRASONIC ---

/**
 * Ermittelt die Distanz vom Roboter zum nächsten Hindernis mit dem
 * Ultraschallsensor.
 * Die Entfernung wird dabei mehrfach gemessen, um Fehler zu minimieren.
 * Negative Entfernungen werden ignoriert (Messfehler).
 *
 * Gibt den Durchschnittswert der Messungen zurück.
 * 
 * Measures the distance of the robot to the nearest object with the
 * ultrasonic sensor. The distance is measured multiple times to minimise
 * the error. Negative distances are ignored.
 *
 * @param int nMeasurements: 
 *     Anzahl der Messungen, Default 3
 *     Number of measurements, default 3
 * @return int nDistanceAvg: 
 *     Entfernung zum nächsten Objekt
 *     distance to the nearest object
 *
 */
int getDistance(int nMeasurements = 3) {
    int nDistanceAvg = 0;

    int nDistance = 0;
    int nDistanceSum = 0;
    
    // Anzahl gültiger (positiver) Messwerte
    // number of valid (positive) measurements
    int nValidValues = 0; 

    for (int i = 0; i <= nMeasurements; i++) {
        nDistance = ultrasonic.read();

        // Hinweis: in früheren Versionen der Ultrasonic Bibliothek wurde
        // distanceRead() verwendet:
        // nDistance = ultrasonic.distanceRead(CM);
        
        // Hint: in earlier versions of the ultrasonic library the 
        // function for measuring the distance was distanceRead().
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
 * Shows the distance to the nearest object by coloring the LEDs 0 and 1.
 * 
 * 150cm - oo: schwarz / black
 *  51cm - 150cm: grün / green 
 *  31cm -  50cm: blau / blue
 *  16cm -  30cm: türkis / turquoise
 *   0cm -  15cm: rot / red
 *
 * @param int nDistance 
 *     Entfernung des Objekts
 *     distance to object in this direction
 *
 */
void showDistance(int nDistance) {

    CRGB color = CRGB::Black;
    if (nDistance > 150) {
      color = CRGB::Black;
    }
    else if ((nDistance <= 150) && (nDistance > 50)) {
      color = CRGB::Red; // wird grün angezeigt / is actually green with some LEDs
    }
    else if ((nDistance <= 50) && (nDistance > 30)) {
      color = CRGB::Blue;
    }
    else if ((nDistance <= 30) && (nDistance > 15)) {
      color = CRGB::DarkTurquoise;
    }
    else if (nDistance > 0) {
      color = CRGB::Green; // wird rot angezeigt / is actually red with some LEDs
    }
    else {
      color = CRGB::Black;
    }

    // Hinweis: Diese Funktion ist zeitkritisch. Wenn mehr als 2 LEDs
    // (bzw. deren Array-Elemente) gesetzt werden, laufen die Motoren
    // nicht mehr.
    
    // HINT: This function is time-sensitive. If more than two LEDs are
    // set, the motors might slow down or stop entirely.

    leds[0] = color;
    leds[1] = color;

    FastLED.show();
}

// --- END FUNCTIONS LED ---


// --- BEGIN FUNCTIONS SERVO ---

/**
 * Stellt die Zielposition für Servo 1 ein und schaltet den Servomotor ein.
 * 
 * Sets the target position for servo 1 and starts the servo.
 *
 */
void startServo1(int targetPos) {
    targetPositionServo1 = targetPos;

    // starte Servo
    // start servo
    servo1.attach(SERVO_1_PIN);
    
    // merken, dass der Servo eingeschaltet ist
    // store the current state of the servo 
    bAttachedServo1 = true; 

    // Setzt die Servo-Position (0..180)
    // Dabei werden die minimale und maximale Position berücksichtigt.
    
    // Sets the servo position (0..180)
    // Minimum and maximum positions are taken into account.
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
 * Sets the target position for servo 2 and starts the servo.
 *
 */
void startServo2(int targetPos) {
    targetPositionServo2 = targetPos;

    // starte Servo
    // start servo
    servo2.attach(SERVO_2_PIN);
    bAttachedServo2 = true;

    // Setzt die Servo-Position (0..180)
    // Dabei werden die minimale und maximale Position berücksichtigt.

    // Sets the servo position (0..180)
    // Minimum and maximum positions are taken into account.
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
 * Stops servo 1
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
 * Stops servo 2
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
 * Cycles servo 1 through four positions continuously. 
 *
 */
void moveServoBackForth() {

    unsigned long timeNow = millis();

    // Positionen sind 60, 90 und 130 Grad
    // positions are 60, 90 and 130 degrees

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
    // Stop servo if the target position or timeout is reached.
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
 * Setzt die Drehrichtung der Motoren ohne die Geschwindigkeit zu ändern.
 * 
 * Sets the direction of the motors without changing the speed.
 *
 * @param int nMotor: 0 - motor A, 1 - motor B
 * @param int nDir: 0 - forward, 1 - backward
 */
void setMotorDir(int nMotor, int nDir) {
    if (MOTOR_A == nMotor) {
        if (FORWARD == nDir) {
            // Motor A vorwärts / forward
            digitalWrite(MOTOR_A_IN1_PIN, HIGH);
            digitalWrite(MOTOR_A_IN2_PIN, LOW);
        }
        else if (BACKWARD == nDir) {
            // Motor A rückwärts / backward
            digitalWrite(MOTOR_A_IN1_PIN, LOW);
            digitalWrite(MOTOR_A_IN2_PIN, HIGH);
        }
    }
    else {
        if (FORWARD == nDir) {
            // Motor B vorwärts / forward
            digitalWrite(MOTOR_B_IN3_PIN, HIGH);
            digitalWrite(MOTOR_B_IN4_PIN, LOW);
        }
        else if (BACKWARD == nDir) {
            // Motor B rückwärts / backward
            digitalWrite(MOTOR_B_IN3_PIN, LOW);
            digitalWrite(MOTOR_B_IN4_PIN, HIGH);
        }
    }
}


/**
 * Einen der beiden Motoren einschalten.
 * 
 * Turn on one of the motors.
 *
 * @param int nMotor: 
 *     Auswahl des Motors, 0 - Motor A, 1 - Motor B
 *     Motor to turn on: 0 - motor A, 1 - motor B
 * 
 * @param int nDir: 
 *     Drehrichtung, 0 - vorwärts, 1 - rückwärts
 *     direction: 0 - forward, 1 - backward
 * @param int nSpeed: 
 *     Geschwindigkeit (0..255), wird durch nMaxSpeed begrenzt
 *     speed (0..255), always limited by nMaxSpeed
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

    // Geschwindigkeit auf 0 .. nMaxSpeed begrenzen
    // limit speed to nMaxSpeed
    nSpeed = max(0, min(nSpeed, nMaxSpeed));

    setMotorDir(nMotor, nDir);

    if (MOTOR_A == nMotor) {
        analogWrite(MOTOR_A_ENABLE_PIN, nSpeed);
    }
    else {
        analogWrite(MOTOR_B_ENABLE_PIN, nSpeed);
    }
}


/**
 * Einen der Motoren anhalten
 * 
 * Stop one of the motors
 * 
 * @param int nMotor: 
 *     Auswahl des Motors, 0 - Motor A, 1 - Motor B
 *     Motor to stop: 0 - motor A, 1 - motor B
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

        // HIGH, um den Motor auszuschalten, 0 zum Bremsen
        // HIGH to stop the motor, 0 to break
        analogWrite(MOTOR_A_ENABLE_PIN, 0);
    }
    else {
        digitalWrite(MOTOR_B_IN3_PIN, LOW);
        digitalWrite(MOTOR_B_IN4_PIN, LOW);

        // HIGH, um den Motor auszuschalten, 0 zum Bremsen
        // HIGH to stop the motor, 0 to break
        analogWrite(MOTOR_B_ENABLE_PIN, 0);
    }
}


/**
 * Beide Motoren anhalten
 * 
 * Stop both motors
 *
 * @param int nDir 
 *     Richtung: FORWARD | BACKWARD
 *     Direction: FORWARD | BACKWARD
 */
void stopMotors(int nDir) {

    startMotors(nDir, 0, nDir, 0);

    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    // HIGH, um den Motor auszuschalten, 0 zum Bremsen
    // HIGH to stop the motor, 0 to break
    analogWrite(MOTOR_A_ENABLE_PIN, 0);

    digitalWrite(MOTOR_B_IN3_PIN, LOW);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);

    // HIGH, um den Motor auszuschalten, 0 zum Bremsen
    // HIGH to stop the motor, 0 to break
    analogWrite(MOTOR_B_ENABLE_PIN, 0);

    nPrevMotorSpeed1 = 0;
    nPrevMotorSpeed2 = 0;
}

/**
 * Setzt die Motorgeschwindigkeiten. Dabei wird die Zielgeschwindigkeit
 * langsam angesteuert, um den Einschaltstrom zu minimieren.
 * 
 * Sets the motor speed. The motors are accellerated (relatively) slowly
 * to minimise the current.
 *
 * @param int nDirA
 *     Richtung Motor A 
 *     Direction of motor A
 * @param int nSpeedA
 *     Geschwindigkeit Motor A
 *     Speed of motor A
 * @param int nDirB
 *     Richtung Motor B 
 *     Direction of motor B
 * @param int nSpeedB
 *     Geschwindigkeit Motor B
 *     Speed of motor B
 *
 * Die Funktion benutzt die globalen Variablen nPrevMotorSpeed1 und nPrevMotorSpeed2
 * 
 * This function also uses the global variables nPrevMotorSpeed1 and nPrevMotorSpeed2
 */
void startMotors(int nDirA, int nSpeedA, int nDirB, int nSpeedB){

    // Drehrichtungen setzen
    // set direction
    setMotorDir(MOTOR_A, nDirA);
    setMotorDir(MOTOR_B, nDirB);

    // MotorSpeed1 und MotorSpeed2 sind die Zielgeschwindigkeiten für
    // jeden Motor.

    // Die Geschwindigkeit wird mit einer Verzögerung erreicht, um den
    // Einschaltstrom gering zu halten. Ohne diese Maßnahme kann der
    // Einschaltstrom so groß werden, dass die Batterien überlastet
    // werden und die Spannung zusammenbricht. In der Folge startet
    // der Arduino neu.

    // nPrevMotorSpeed1 und nPrevMotorSpeed2 speichern die vorherige Geschwindigkeit
    // der beiden Motoren. Diese kann kleiner, größer oder gleich der
    // gewünschten Zielgeschwindigkeit sein.

    // nDelay hängt von der Differenz zwischen aktueller und Zielgeschwindigkeit
    // ab.
    
    
    // MotorSpeed1 and MotorSpeed2 are the target speeds for the two motors.
    
    // The motors are accellerated slowly to avoid power loss. Without 
    // this, the turn-on current can exceed the maximum current of the
    // batteries and cause the voltage to drop. This in turn causes the 
    // Arduino to reboot. 
    
    // nPrevMotorSpeed1 und nPrevMotorSpeed2 stoire the previous speed 
    // of each motor. This can be less than, equal to or greater than 
    // the desired target speed.
    
    // The delay nDelay is calculated from the difference of current
    // and target speed.

    int nDelay = 0;
    int nDiffA = abs(nSpeedA - nPrevMotorSpeed1);
    int nDiffB = abs(nSpeedB - nPrevMotorSpeed2);
    if ((nDiffA > 50) || (nDiffB > 50)) {
        nDelay = 1;
    }
    else if ((nDiffA > 120) || (nDiffB > 120)) {
        nDelay = 2;
    }
    else if ((nDiffA > 180) || (nDiffB > 180)) {
        nDelay = 3;
    }
    else {
        nDelay = 4;
    }

    int nCurMotorSpeed1 = nPrevMotorSpeed1;
    int nCurMotorSpeed2 = nPrevMotorSpeed2;

    while ((nCurMotorSpeed1 != nSpeedA) || (nCurMotorSpeed2 != nSpeedB)) {
        if (nCurMotorSpeed1 > nSpeedA) {
            nCurMotorSpeed1--;
        }
        if (nCurMotorSpeed1 < nSpeedA) {
            nCurMotorSpeed1++;
        }
        if (nCurMotorSpeed2 > nSpeedB) {
            nCurMotorSpeed2--;
        }
        if (nCurMotorSpeed2 < nSpeedB) {
            nCurMotorSpeed2++;
        }
        analogWrite(MOTOR_A_ENABLE_PIN, nCurMotorSpeed1);
        analogWrite(MOTOR_B_ENABLE_PIN, nCurMotorSpeed2);

        // warten
        // wait
        delay(nDelay);
    }

    delay(20);

    nPrevMotorSpeed1 = nCurMotorSpeed1;
    nPrevMotorSpeed2 = nCurMotorSpeed2;
}


/**
 * Dreht den Roboter nach links oder rechts, indem ein Motor für nDelay
 * Millisekunden vorwärts und der andere rückwärts gedreht wird.
 * 
 * Turns the robot left or right by driving one motor and the other one 
 * backward for nDelay milliseconds.
 *
 * @param int nDir: 
 *     Drehrichtung, 0 - links / 1 - rechts
 *     direction, 0 - left / 1 - right
 * @param int nDelay: 
 *     Dauer der Drehung in ms
 *     duration of the turning (ms)
 *
 */
void turnRobot(int nDir, int nDelay) {
    stopMotors(FORWARD);
    delay(100);

    int nSpeed = 128;

    if (LEFT == nDir) {
        startMotors(FORWARD, nSpeed, BACKWARD, nSpeed);
    }
    else {
        startMotors(BACKWARD, nSpeed, FORWARD, nSpeed);
    }

    delay(nDelay);
    stopMotors(FORWARD);
}

// --- END FUNCTIONS MOTOR ---


// --- BEGIN FUNCTIONS DO_BATTLE ---

// Diese Funktionen sind abhängig von den Abschnitten MOTOR, SERVO und ULTRASONIC

// These function depend on the sections MOTOR, SERVO and ULTRASONIC

/**
 * Misst den Abstand zum nächsten Objekt in Richtung nAngle
 * 
 * Measures the distance to the nearest object in the direction nAngle.
 * Sets the servo to nAngle and measures the distance to the nearest 
 * object.
 *
 * @param int nAngle: 
 *     Winkel
 *     Angle
 * @return int nDistance: 
 *     Abstand zum Objekt in cm
 *     Distance to the nearest object (cm)
 *
 */
int getDistanceDir(int nAngle) {

    // Servo auf den Winkel ausrichten
    // turn servo to position nAngle
    startServo2(nAngle);
    delay(DETACH_DELAY_SERVO_2);
    stopServo2();

    // Abstand messen
    // measure the distance
    int nDistance = getDistance();

    return nDistance;
}


/**
 * Misst den Abstand zum nächsten Objekt in verschiedenen Richtungen.
 * Liefert die Richtung (Winkel) des nächsten Objekts zurück.
 *
 * Falls es mehrere nächste Objekte gibt, wird der kleinere Winkel
 * zurückgegeben.
 *
 * Falls kein Objekt weniger als nMaxDistance cm entfernt ist, wird ein
 * Winkel von 90 Grad zurückgeliefert.
 * 
 * Measures the distance to the nearest objects in several directions.
 * Returns the direction (angle) of the nearest object.
 * 
 * If more than one such object exists, the smaller angle is returned.
 * 
 * If no object is nearer than nMaxDistance cm, an angle of 90 degrees
 * is returned.
 *
 * @param int nMaxDistance: 
 *     max. Entfernung eines Objekts, Default 100cm
 *     maximum distance to the nearest object, default 100cm
 * @return int nAngle: 
 *     Winkel zum nächsten Objekt oder 90 Grad, falls kein Objekt in 
 *     der Nähe ist.
 *     Angle of the nearest object or 90 degrees, if no object is found.
 *
 */
int getDirectionOfNearestObject(int nMaxDistance = 100) {
    int nMinAngle = 30; // 20
    int nMaxAngle = 150; // 160
    int nNearestObjectAngle = 90; // default
    int nNearestObjectDistance = 999;
    int nDistance = 0;

    // mehrere Winkel durchlaufen und jeweils den Abstand messen
    // measure the distance in several directions
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


/**
 * Die Funktion sucht nach Objekten in der Nähe und greift sie an.
 * 
 * Die Funktion durchläuft verschiedene Zustände ("Endlicher Automat").
 * Jeder Zustand ist mit einer Handlung verbunden.
 * Der Übergang in einen anderen Zustand wird über interne und externe
 * Bedingungen bestimmt.
 *
 * Search for objects and attack them.
 *
 * The function cycles through several states ("state machine"). Each
 * state represents an action for the attack. The transition between 
 * the states is determined by internal and external conditions. 
 *
 */
void doBattle() {
    // Standardeinstellungen
    // default settings
    int nDistance = getDistance();
    int nAngleOfOpponent = 90;
    int nSpeed = 0;

    // Entfernung zum nächsten Objekt mit LEDs anzeigen
    // show the distance to the nearest object on the LEDs
    showDistance(nDistance);

#ifdef HASOLED
    // Entfernung zum nächsten Objekt auf dem OLED Display anzeigen
    // show the distance to the nearest object on the OLED display
    showDistanceOLED(nDistance);
#endif

    // Zustand setzen

    // Entfernung zum Ziel < 20cm? Dann Attacke
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

	// Set state
	
	// Distance to target < 20cm? Then attack!
	// Otherwise: 
	//     - look left and right to find a target
	// 	   - turn to the target
	//     - approach the target
	
	// nBattleState:
	// 0 - lurk, find opponent and turn to face it
	// 1 - straight forward attack
	// 2 - swerve left and attack
	// 3 - swerve right and attack
	// 4 - retreat
	// 5 - start searching
	// 6 - stop
	// 7 - drive and search 
	
    switch (nBattleState) {
        case 0:
            // lauern, Gegner suchen, Robat auf Gegner ausrichten
            
            // lurk, find opponent and turn to face it

            // Abstand in verschiedene Richtungen messen und zum nächsten
            // Objekt drehen
            // measure distance to objects in several directions and
            // turn to the nearest one
            nAngleOfOpponent = getDirectionOfNearestObject();
            nDistance = getDistanceDir(nAngleOfOpponent);

            // Servo zurück auf 90 Grad (geradeaus)
            // set servo to position 90 degrees (straight on)
            startServo2(90);
            delay(DETACH_DELAY_SERVO_2);
            stopServo2();

            if (nAngleOfOpponent < 90) {
                // drehe den Roboter nach links
                // turn robot left
                turnRobot(LEFT, 4 * (90 - nAngleOfOpponent));
            }
            else if (nAngleOfOpponent > 90) {
                // drehe den Roboter nach rechts
                // turn robot right
                turnRobot(RIGHT, 4 * (nAngleOfOpponent - 90));
            }
            else {
                // nächstes Objekt liegt recht voraus, Roboter nicht drehen
                // next object is straight on, don't turn the robot
            }

            if (nDistance < 20) {
                // Angriff
                // attack
                nBattleState = random(3) + 1; // 1, 2 oder 3
            }
            else {
                // Suchfahrt geradeaus
                // start searching
                nBattleState = 5;
            }
            break;
        case 1:
            // Angriff geradeaus
            // straight forward attack

			// Anzahl durchgeführter Angriffe
			// number of attacks
            nAttackCounter++; 

			// volle Kraft voraus
			// full speed ahead
            nSpeed = nMaxSpeed; 
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(1000);

            stopMotors(FORWARD);

			// beim ersten Angriff nur vorwärts fahren
			// only go forward on the first attack
            if (1 == nAttackCounter) {
                // lauern
                // lurk
                nBattleState = 0; 
            }
            else {
				// zurückziehen
				// retreat
                nBattleState = 4; 
            }

            break;

        case 2:
            // Angriff mit Schlenker links
			// swerve left and attack
			
			// Anzahl durchgeführter Angriffe erhöhen
			// increase number of attacks already conducted
            nAttackCounter++; 

			// halbe Kraft voraus
			// half speed ahead
            nSpeed = int(nMaxSpeed / 2); 

            // drehe den Roboter nach links
            // turn robot left
            turnRobot(LEFT, 250);
            
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(500);

            // drehe den Roboter nach rechts
            // turn robot right
            turnRobot(RIGHT, 250);

            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(500);

            // drehe den Roboter nach rechts
            // turn robot right
            turnRobot(RIGHT, 250);

			// volle Kraft voraus
			// full speed ahead
            nSpeed = nMaxSpeed; 
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(800);

            stopMotors(FORWARD);

			// zurückziehen
			// retreat
            nBattleState = 4; 
            
            break;

        case 3:
            // Angriff mit Schlenker rechts
			// swerve right and attack

			// Anzahl durchgeführter Angriffe erhöhen
			// increase number of attacks already conducted			
            nAttackCounter++; 

			// halbe Kraft voraus
			// half speed ahead
            nSpeed = int(nMaxSpeed / 2); 


            // drehe den Roboter nach rechts
            // turn robot right
            turnRobot(RIGHT, 250);
            
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(500);

            // drehe den Roboter nach links
            // turn robot left
            turnRobot(LEFT, 250);

            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(500);

            // drehe den Roboter nach links
            // turn robot left
            turnRobot(LEFT, 250);

            nSpeed = nMaxSpeed;
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
            delay(800);

            stopMotors(FORWARD);

			// lauern
			// lurk
            nBattleState = 0; 

            break;

        case 4:
            // Zurückweichen
            // retreat

            nSpeed = 100;
            startMotors(BACKWARD, nSpeed, BACKWARD, nSpeed);
            delay(800);

            stopMotors(BACKWARD);

            nBattleState = 0;
            break;

        case 5:
            // Suchfahrt geradeaus starten
            // start searching

            // Beginn der Suchfahrt merken
            // remember the time when we last searched for an opponent
            timeOfLastDistanceMeasurement = millis();

            nSpeed = 80;
            startMotors(FORWARD, nSpeed, FORWARD, nSpeed);

            nBattleState = 7;
            break;

        case 6:
            // anhalten
            // stop
            stopMotors(FORWARD);

            break;

        case 7:
            // Suchfahrt geradeaus
            // drive and search

            nDistance = getDistanceDir(90);

            if (nDistance < 20) {
                // Angriff
                // attack
                nBattleState = 1;
            }

            // Zustandsübergang, wenn "lange" kein Gegner gesehen wurde
            // change the state if we didn't see an opponent for some time
            if (millis() - timeOfLastDistanceMeasurement > 1000) {
				
                // anhalten und erneut nach Gegnern suchen
                // stop and search for opponents again
                stopMotors(FORWARD);

                nBattleState = 0;
            }
            break;
    }
}

// --- END FUNCTIONS DO_BATTLE ---


// --- BEGIN FUNCTIONS AVOID_OBSTACLES ---

// Diese Funktionen sind abhängig von den Abschnitten MOTOR, SERVO und ULTRASONIC
// These functions depend on the sections MOTOR, SERVO and ULTRASONIC

/**
 * Die Funktion versucht, den Roboter so zu steuern, dass er nicht
 * gegen Hindernisse prallt.
 * 
 * This function lets the robot avoid obstacles. 
 *
 */
void avoidObstacles() {

    int nDistance = getDistance();

    if (DEBUG) {
        // Serial.print("Distance to object: ");
        Serial.println(nDistance);
    }

    showDistance(nDistance);

#ifdef HASOLED
    // Entfernung zum nächsten Objekt auf dem OLED Display anzeigen
    // show the distance to the nearest object on the OLED display
    showDistanceOLED(nDistance);
#endif

    int nSpeed = 0;

    // Kollisionserkennung über die Schalter links und rechts vorne
    // (falls vorhanden) 
    // collision detection with bumper switches of the robot 
    // (if they exist)
    int bBumper1 = digitalRead(BUMPER1_PIN);
    int bBumper2 = digitalRead(BUMPER2_PIN);
    
    if (digitalRead(BUMPER1_PIN) == LOW) {
        // Zusammenstoß mit Hindernis erkannt. Ausweichen
        // collision detected, evade

        stopMotors(FORWARD);

        // Zustand durch Töne signalisieren
        // play sound
        TimerFreeTone(TONE_PIN, NOTE_A3, 160);
        TimerFreeTone(TONE_PIN, NOTE_PAUSE, 160);
        TimerFreeTone(TONE_PIN, NOTE_A4, 80);
        TimerFreeTone(TONE_PIN, NOTE_PAUSE, 80);
        TimerFreeTone(TONE_PIN, NOTE_A5, 80);

        delay(250);
        startMotors(BACKWARD, 128, BACKWARD, 128);
        delay(250);
        stopMotor(MOTOR_A);
        delay(1500);
        stopMotor(MOTOR_B);
        
    }
    else if (digitalRead(BUMPER2_PIN) == LOW) {
        // Zusammenstoß mit Hindernis erkannt. Ausweichen
		// collision detected, evade

        stopMotors(FORWARD);

        // Zustand durch Töne signalisieren
        // play sound
        TimerFreeTone(TONE_PIN, NOTE_A5, 80);
        TimerFreeTone(TONE_PIN, NOTE_PAUSE, 80);
        TimerFreeTone(TONE_PIN, NOTE_A4, 80);
        TimerFreeTone(TONE_PIN, NOTE_PAUSE, 160);
        TimerFreeTone(TONE_PIN, NOTE_A3, 160);

        delay(250);
        startMotors(BACKWARD, 128, BACKWARD, 128);
        delay(500);
        stopMotor(MOTOR_B);
        delay(1250);
        stopMotor(MOTOR_A);
    }
    else if (nDistance > 15) {
        // ausreichender Abstand zum nächsten Objekt
        // geradeaus fahren
        // Geschwindigkeit ist abhängig von der Entfernung zum nächsten
        // Hindernis
        
        // enough free space in front of the robot
        // drive ahead
        // The speed depends on the distance to the nearest object.
        
        // Bilde die Entfernungen 15cm bis 150cm auf 
        // die Geschwindigkeiten 80 bis nMaxSpeed ab.
		// map the distances 15cm to 150cm to the speeds 80 to nMaxSpeed
        nSpeed = min(map(nDistance, 15, 150, 80, nMaxSpeed), nMaxSpeed);
        startMotors(FORWARD, nSpeed, FORWARD, nSpeed);
    }
    else {
        // Kollisionsgefahr. Anhalten
        // imminent collision, stop!
        stopMotors(FORWARD);

        // Zustand durch Töne signalisieren
        // play sound
        TimerFreeTone(TONE_PIN, NOTE_A4, 80);
        TimerFreeTone(TONE_PIN, NOTE_PAUSE, 80);
        TimerFreeTone(TONE_PIN, NOTE_A3, 200);

        // Abstand in verschiedene Richtungen messen und in eine freie
        // Richtung drehen.

		// find a free path
		
        // links frei?
        // check left side
        startServo2(60);
        delay(DETACH_DELAY_SERVO_2);
        stopServo2();
        nDistance = getDistance();

        if (nDistance > 15) {
            // nach links drehen
            // turn left
            turnRobot(LEFT, 200);
        }
        else {
            // rechts frei?
            // check right side
            startServo2(120);
            delay(DETACH_DELAY_SERVO_2);
            stopServo2();
            nDistance = getDistance();

            if (nDistance > 15) {
                // nach rechts drehen
                // turn right
                turnRobot(RIGHT, 200);
            }
            else {
                // umdrehen
                // turn around
                turnRobot(RIGHT, 1000);
            }
        }

        // Servo mit Ultraschallsensor wieder nach vorne ausrichten
        // turn servo to 90 degree position
        startServo2(90);
        delay(DETACH_DELAY_SERVO_2);
        stopServo2();
    }
}

// --- END FUNCTIONS AVOID_OBSTACLES ---


// --- BEGIN FUNCTIONS MANUAL_CONTROL ---

// Diese Funktionen sind abhängig von den Abschnitten MOTOR und JOYSTICK
// These functions depend on the sections MOTOR and JOYSTICK

/**
 * Manuelle Steuerung des RoBat, über die Joystick-Positionen wird die
 * Geschwindigkeit und Drehrichtung der Motoren gesteuert.
 * 
 * Manual control of the robot. Control the speed and direction of 
 * the motors with the joystick.
 *
 */
void manualControl() {

    // Joystick Position einlesen
    // read joystick position

    nJoyPosV = analogRead(JOY_VERTICAL_PIN);
    nJoyPosH = analogRead(JOY_HORIZONTAL_PIN);

    // Der Joystick liefert Werte zwischen 0 und 1023 in X- und Y-Richtung.
    // Wenn der Joystick in der Mittelstellung ist, müsste er theoretisch
    // einen Wert von 512 für beide Richtungen liefern. In der Praxis
    // weichen die Werte davon ab. Wir definieren deshalb alle Werte
    // zwischen JOY_MIDDLE_MIN und JOY_MIDDLE_MAX als Mittelstellung.
    
    // The joystick returns values between 0 and 1023 in horizontal (X)
    // and vertical (Y) 

    int nMotorDir = STOP; // default

    // Wird der Joystick nach hinten gedrückt (nJoyPosV < JOY_MIDDLE_MIN),
    // soll sich der Roboter rückwärts bewegen. Wird der Joystick nach
    // vorne gedrückt (nJoyPosV > JOY_MIDDLE_MAX), bewegt sich der Roboter
    // vorwärts.

    // Wenn der Joystick nicht angeschlossen ist, liefern die Eingänge
    // zufällige Werte. In diesem Fall sollten die beiden Eingänge auf 
    // GND gelegt werden, dann liefern sie einen Wert von 0.
    
    // If the joystick is pulled backwards  (nJoyPosV < JOY_MIDDLE_MIN), 
    // the robot should also drive backwards. If the joystick is pushed 
    // forward (nJoyPosV > JOY_MIDDLE_MAX), the robot should move 
    // forwards, too.
    
    // If the joystick is not connected, the input will be random values.
    // In this case, the pins should be connected to ground (GND). They 
    // will then return a value of zero (0).

    if ((0 == nJoyPosV) && (0 == nJoyPosH)) {
        // kein Joystick angeschlossen
        // joystick not connected
        nMotorSpeed1 = 0;
        nMotorSpeed2 = 0;
    }
    else if (nJoyPosV < JOY_MIDDLE_MIN) {
        // rückwärts
        // backward
        nMotorDir = BACKWARD;

        // Motorgeschwindigkeit ist abhängig davon, wie weit der Joystick
        // gedrückt wird.
        // the motor speed depends on the values from the joystick

        // für die Rückwärtsfahrt werden die Werte umgekehrt
        // the values are inverted if the robots drives backward

		// negative Zahlen
		// negative numbers
        nJoyPosV = nJoyPosV - JOY_MIDDLE_MIN; 
        
        // positiv machen
        // invert the numbers
        nJoyPosV = nJoyPosV * -1;  

        nMotorSpeed1 = map(nJoyPosV, 0, JOY_MIDDLE_MIN, 0, nMaxSpeed);
        nMotorSpeed2 = map(nJoyPosV, 0, JOY_MIDDLE_MIN, 0, nMaxSpeed);
    }
    else if (nJoyPosV > JOY_MIDDLE_MAX) {
        // vorwärts
        // forward
        nMotorDir = FORWARD;

        // Motorgeschwindigkeit ermitteln
        // calculate motor speed
        nMotorSpeed1 = map(nJoyPosV, JOY_MIDDLE_MAX, 1023, 0, nMaxSpeed);
        nMotorSpeed2 = map(nJoyPosV, JOY_MIDDLE_MAX, 1023, 0, nMaxSpeed);
    }
    else {
        // anhalten
		// stop
		
        nMotorSpeed1 = 0;
        nMotorSpeed2 = 0;
    }

    // Steuerung links und rechts
	// left and right control
	
    // Die horizontale Joystickposition beeinflusst die Motorgeschwindigkeit
    // the horizontal position of the joystick also influences the motor speed

    if ((0 == nJoyPosV) && (0 == nJoyPosH)) {
        // kein Joystick angeschlossen
        // joystick not connected
        nMotorSpeed1 = 0;
        nMotorSpeed2 = 0;
    }
    else if (nJoyPosH < JOY_MIDDLE_MIN) {
        // links
		// left
		
        // Bei der Bewegung nach links werden die Werte umgekehrt
        // the values are inverted if the robot turns left

		// negative Zahlen
		// negative numbers
        nJoyPosH = nJoyPosH - JOY_MIDDLE_MIN; 
        
        // invert the numbers
        // positiv machen
        nJoyPosH = nJoyPosH * -1;  

        // Die Werte vom Joystick liegen zwischen 0 und 1023. Sie werden
        // auf den Bereich der Motorgeschwindigkeit von 0 bis nMaxSpeed
        // abgebildet.

		// The joystick position values are between 0 and 1023. They are mapped
		// on the range 0 to nMaxSpeed.
        nJoyPosH = map(nJoyPosH, 0, JOY_MIDDLE_MIN, 0, nMaxSpeed);

        // Motorgeschwindigkeiten setzen
        // set motor speed
        nMotorSpeed1 = nMotorSpeed1 - nJoyPosH;
        nMotorSpeed2 = nMotorSpeed2 + nJoyPosH;

        // Motorgeschwindigkeit im erlaubten Bereich halten
        // keep motor speed within the allowed limits
        if (nMotorSpeed1 < 0) {
            nMotorSpeed1 = 0;
        }
        if (nMotorSpeed2 > nMaxSpeed) {
            nMotorSpeed2 = nMaxSpeed;
        }
    }
    else if (nJoyPosH > JOY_MIDDLE_MAX) {
        // rechts
        // right

        // Die Werte vom Joystick liegen zwischen 0 und 1023. Sie werden
        // auf den Bereich der Motorgeschwindigkeit von 0 bis nMaxSpeed
        // abgebildet.

		// The joystick position values are between 0 and 1023. They are mapped
		// on the range 0 to nMaxSpeed.

        nJoyPosH = map(nJoyPosH, JOY_MIDDLE_MAX, 1023, 0, nMaxSpeed);

        // Motorgeschwindigkeiten setzen
        // set motor speed
        nMotorSpeed1 = nMotorSpeed1 + nJoyPosH;
        nMotorSpeed2 = nMotorSpeed2 - nJoyPosH;

        // Motorgeschwindigkeit im erlaubten Bereich halten
        // keep motor speed within the allowed limits
        if (nMotorSpeed1 > nMaxSpeed) {
            nMotorSpeed1 = nMaxSpeed;
        }
        if (nMotorSpeed2 < 0) {
            nMotorSpeed2 = 0;
        }
    }

    // bei sehr niedrigen Motorgeschwindigkeiten laufen die Motoren
    // nicht an, manchmal entsteht ein summendes Geräusch. Das wird hier
    // unterdrückt.

	// At very low motor speeds, the motors won't start at all. Sometimes
	// a humming or buzzing noise can be heard. We suppress this by setting
	// the motor speed to 0 (zero) if it is below a threshold.
	
    if (nMotorSpeed1 < 8) {
        nMotorSpeed1 = 0;
    }
    if (nMotorSpeed2 < 8) {
        nMotorSpeed2 = 0;
    }

    // Motorgeschwindigkeit mit den LEDs anzeigen
	// show the motor speed on the LEDs
    if ((nMotorSpeed1 == 0) || (nMotorSpeed2 == 0)) {
        leds[0] = CRGB::White;
        leds[1] = CRGB::White;
    }
    else if (FORWARD == nMotorDir) {
        leds[0] = CRGB::Blue;
        leds[1] = CRGB::Blue;
    }
    else if (BACKWARD == nMotorDir) {
        leds[0] = CRGB::Green;
        leds[1] = CRGB::Green;
    }

    FastLED.show();

    // Motorgeschwindigkeiten setzen
    // set motor speed
    startMotors(nMotorDir, nMotorSpeed1, nMotorDir, nMotorSpeed2);

}

// --- END FUNCTIONS MANUAL_CONTROL ---


// --- BEGIN FUNCTIONS SERIAL COMMANDS ---

/**
 * Verarbeitet Kommandos, die über die serielle Schnittstelle
 * empfangen werden. 
 * 
 * Processes commands received on the serial interface.
 * 
 */
void doSerialCommand() {

  // Im SERIALCMD Modus immer zuerst prüfen, ob der Timeout abgelaufen ist.
  // Der Timeout verhindert, dass der Robotor unkontrollierbar wird, wenn
  // die serielle Verbindung unterbrochen wird. Nach Ablauf des Timeouts
  // hält der Roboter an.
  
  // To prevent the robot from becoming uncontrollable if the serial 
  // connection breaks, we introduce a timeout. When no commands are 
  // received within a certain period of time, the robot stops.
  
  if (SERIALCMD == nMode) {
    if (millis() > nLastCmd + nMotorTimeout) {
      Serial.println("timeout stop");
      // Motoren anhalten
      // stop motors
      stopMotor(MOTOR_A);
      stopMotor(MOTOR_B);

      nLastCmd = millis();
      return;
    }
  }

  // prüfen, ob ein serielles Kommando bereitsteht
  // check if a new serial command is available
  if (bSerialComplete) {

    if (DEBUG) {
      Serial.print("rec:");
      Serial.println(strSerialInput);
    }
    
    int nSpeed = 0;
    // Anlaufgeschwindigkeit der Motoren
    // starting speed of the motors
    int nMinSpeed = 50; 
    int nMotor = 0;
    int nDir = FORWARD;
    boolean bSetMotors = false;

    // Modus setzen
    // set mode
    if (strSerialInput.substring(0,9) == "serialcmd") {
      Serial.println("serialcmd mode");
      nMode = SERIALCMD;

      // Motoren anhalten
      // stop motors
      nSpeed = 0;
      bSetMotors = true;

#ifdef HASOLED
      // Betriebsmodus auf dem OLED Display anzeigen
      // Show current mode of operation on the display
      showMode();
#endif
      
    }
    else if (strSerialInput.substring(0,6) == "manual") {
      Serial.println("manual mode");
      nMode = MANUAL;

      // Motoren anhalten
      // stop motors
      nSpeed = 0;
      bSetMotors = true;

#ifdef HASOLED
      // Betriebsmodus auf dem OLED Display anzeigen
      // Show current mode of operation on the display
      showMode();
#endif
    
    }
    else if (strSerialInput.substring(0,6) == "battle") {
      Serial.println("battle mode");
      nMode = BATTLE;
      
      // Motoren anhalten
      // stop motors
      nSpeed = 0;
      bSetMotors = true;

#ifdef HASOLED
      // Betriebsmodus auf dem OLED Display anzeigen
      // Show current mode of operation on the display
      showMode();
#endif
    
    }
    else if (strSerialInput.substring(0,10) == "autonomous") {
      Serial.println("autonomous mode");
      nMode = AUTONOMOUS;
      
      // Motoren anhalten
      // stop motors
      nSpeed = 0;
      bSetMotors = true;

#ifdef HASOLED
      // Betriebsmodus auf dem OLED Display anzeigen
      // Show current mode of operation on the display
      showMode();
#endif
    
    }
    
    // Distanz zum nächsten Objekt mit dem Ultraschallsensor messen
    // measure the distance to the closest object
    else if (strSerialInput.substring(0,4) == "dist") {
      int nDistance = getDistance();
      Serial.println(nDistance);
    }
    
    // linker Motor vorwärts
    // left motor forward
    else if (strSerialInput.substring(0,2) == "lf") {
      nSpeed = strSerialInput.substring(2,5).toInt();
      nSpeed = max(min(nSpeed, nMaxSpeed), nMinSpeed);
      nMotor = MOTOR_A;
      nDir = FORWARD;
      bSetMotors = true;
    }
    
    // linker Motor rückwärts
    // left motor backward
    else if (strSerialInput.substring(0,2) == "lb") {
      nSpeed = strSerialInput.substring(2,5).toInt();
      nSpeed = max(min(nSpeed, nMaxSpeed), nMinSpeed);
      nMotor = MOTOR_A;
      nDir = BACKWARD;
      bSetMotors = true;
    }
    
    // rechter Motor vorwärts
    // right motor forward
    else if (strSerialInput.substring(0,2) == "rf") {
      nSpeed = strSerialInput.substring(2,5).toInt();
      nSpeed = max(min(nSpeed, nMaxSpeed), nMinSpeed);
      nMotor = MOTOR_B;
      nDir = FORWARD;
      bSetMotors = true;
    }
    
    // rechter Motor rückwärts
    // right otor backward
    else if (strSerialInput.substring(0,2) == "rb") {
      nSpeed = strSerialInput.substring(2,5).toInt();
      nSpeed = max(min(nSpeed, nMaxSpeed), nMinSpeed);
      nMotor = MOTOR_B;
      nDir = BACKWARD;
      bSetMotors = true;
    }
    
    // Motoren anhalten
    // stop motors
    else if (strSerialInput.substring(0,2) == "st") {
      nSpeed = 0;
      bSetMotors = true;
    }

    if (true == bSetMotors) {
      // Die Motoren werden in jedem Modus angehalten,
      // aber nur im SERIALCMD Modus gestartet
      // The motors are stopped in any mode, but started only in 
      // SERIALCMD mode
      if (nSpeed == 0) {
        stopMotor(MOTOR_A);
        stopMotor(MOTOR_B);
      }
      else if ((nSpeed > 0) && (nMode == SERIALCMD)) {
        startMotor(nMotor, nDir, nSpeed);
      }

    }

    // Kommando löschen
    // clear command buffer
    strSerialInput = "";
    bSerialComplete = false;
    nLastCmd = millis();
  }


}

// --- END FUNCTIONS SERIAL COMMANDS ---


void setup() {

// --- BEGIN SETUP GENERAL ---

    // Serielle Schnittstelle konfigurieren
    // configure the serial interface
    Serial.begin(SERIALSPEED);

// --- END SETUP GENERAL ---


// --- BEGIN SETUP OLED ---

#ifdef HASOLED

// mit I2C-Adresse 0x3c initialisieren
// initialize with the I2C addr 0x3C 

display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

display.clearDisplay();

// set text color / Textfarbe setzen
display.setTextColor(WHITE);

// set text size / Textgroesse setzen
display.setTextSize(1);

// set text cursor position / Textstartposition einstellen
display.setCursor(1,0);

// Text anzeigen / show welcome message
for (int i=0; i<4; i++) {
  display.println(displaylines[i]);
}
display.display();

#endif

// --- END SETUP OLED ---


// --- BEGIN SETUP SERIAL COMMAND ---

// Puffer für eingehende Kommandos
// buffer for incoming serial commands
strSerialInput.reserve(200);

// --- END SETUP SERIAL COMMAND ---


// --- BEGIN SETUP BUMPER ---

    // Kollisionserkennung
    // collision detection
    pinMode(JOYSTICK_SWITCH_PIN, INPUT_PULLUP);
    pinMode(BUMPER1_PIN, INPUT_PULLUP);
    pinMode(BUMPER2_PIN, INPUT_PULLUP);

// --- END SETUP BUMPER ---


// --- BEGIN SETUP SERVO ---

    pinMode(SERVO_1_PIN, OUTPUT);
    pinMode(SERVO_2_PIN, OUTPUT);

    // Servos beim Start hin und her bewegen (Funktionstest)
	// move servo back and forth at startup (check)
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

    // in Mittelstellung ausrichten
    // move servo to middle position (90 degrees)
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

// --- END SETUP SERVO ---


// --- BEGIN SETUP BUZZER ---

    // Startmelodie abspielen
    // play melody
    
    // alle Noten im Array durchlaufen
    // loop over all the notes in melody[]
    for (int thisNote = 0; thisNote < melody_length; thisNote++) { 
		// Note abspielen
		// play note
        TimerFreeTone(TONE_PIN, melody[thisNote], duration[thisNote]); 
        // kurze Pause zwischen den Noten
        // short gap between notes
        delay(50); 
    }

// --- END SETUP BUZZER ---


// --- BEGIN SETUP MOTOR ---

    // Pins für die Motorkontrolle als OUTPUT definieren
    // set motor pins to OUTPUT mode
    pinMode(MOTOR_A_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_B_ENABLE_PIN, OUTPUT);
    pinMode(MOTOR_A_IN1_PIN, OUTPUT);
    pinMode(MOTOR_A_IN2_PIN, OUTPUT);
    pinMode(MOTOR_B_IN3_PIN, OUTPUT);
    pinMode(MOTOR_B_IN4_PIN, OUTPUT);

    pinMode(MOTOR_STANDBY_PIN, OUTPUT);

    // Motor Treiber einschalten
    // turn on the motor driver
    digitalWrite(MOTOR_STANDBY_PIN, HIGH);

    // Motoren aus und Richtung auf vorwärts einstellen
	// stop motors and set direction to FORWARD
	
    // Motor A
    digitalWrite(MOTOR_A_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);

    // Motor B
    digitalWrite(MOTOR_B_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_B_IN3_PIN, HIGH);
    digitalWrite(MOTOR_B_IN4_PIN, LOW);

// --- END SETUP MOTOR ---


// --- BEGIN SETUP MODE ---

    // # Betriebsmodus bestimmen
    //
    // Es gibt vier Betriebsmodi:
    //
    // - MANUAL: Steuerung mit dem Joystick
    // - AUTONOMOUS: RoBat fährt und versucht Hindernisse zu vermeiden
    // - BATTLE: RoBat sucht Hindernisse und rammt sie
    // - SERIALCMD: RoBat wird über die serielle Schnittstelle mit Kommandos gesteuert

    // Beim Einschalten wird der Betriebsmodus über den Abstand zum nächsten Objekt
    // und den Zustand des Joystick-Knopfs bzw. der Fühler bestimmt.

    // Der Anschluss des Joysticks ist optional.
    // Der Ultraschallsensor kann fehlen. In diesem Fall liefert die Abfrage der
    // Entfernung getDistance() 0 zurück.

    // Beim Einschalten soll RoBat aktiv sein, wenn keine Handsteuerung gewünscht wird.
    // Ohne Ultraschallsensor ist autonomes Fahren nur sinnvoll, wenn zwei Fühler
    // vorhanden sind, die Kollisionen feststellen können.

    // Der MANUAL Modus kann mit dem Kommando "sercmd" über die serielle Schnittstelle
    // in den Modus SERIALCMD überführt werden.

    // Joystick-Knopf gedrückt: MANUAL
    // Abstand > 0 und < 10cm: MANUAL
    // Abstand >= 10cm und < 50cm: Battle-Modus
    // Abstand >= 50cm und < 100cm: Hindernisvermeidung
    // sonst: Steuerung über serielle Schnittstelle
    
    // falls der Joystick Button beim Einschalten gedrückt wird, ist der
    // Modus ebenfalls MANUAL

    // Wenn beide Fühler vorhanden und gedrückt sind, wird der Modus BATTLE verwendet.

	// Determine the mode of operation
	//
	// There are four modes of operation:
	// 
	// - manual control (MANUAL)
	// - obstacle avoidance (AUTONOMOUS)
	// - Hebocon / battle (BATTLE)
	// - external control via the serial interface (SERIAL)
	// 
	// When the robot is turned on, the operation mode will be determined
	// by the distance to the closest object, the joystick button and
	// the bumper switches.
	
	// Connecting the joystick is optional. The ultrasonic sensor may be 
	// missing. In this case, the distance returned by getDistance() 
	// will be 0 (zero).
	// Without the ultrasonic sensor, autonomous driving only makes sense
	// if the bumper switches are connected. Otherwise, the robot would be 
	// completely blind.
	
	// SERIALCMD mode can be selected by sending "sercmd" over the serial 
	// interface to the robot.

	// Determining the mode of operation
	// joystick button held down at power-on: MANUAL
	// distance > 0 and < 10cm: MANUAL
	// distance >= 10cm und < 50cm: BATTLE
    // distance >= 50cm und < 100cm: obstacle avoidance
    // otherwise: SERIALCMD

	// If both bumper switches are pressed, the robot also goes into
	// BATTLE mode.
	
    int nDistance = getDistance();
    int bButtonPressed = digitalRead(JOYSTICK_SWITCH_PIN);
    int bBumper1 = digitalRead(BUMPER1_PIN);
    int bBumper2 = digitalRead(BUMPER2_PIN);

    if (LOW == bButtonPressed) {
        nMode = MANUAL;
    }
    else if ((nDistance > 0) && (nDistance < 10)) {
        nMode = MANUAL;
    }
    else if ((nDistance >= 10) && (nDistance < 50)) {
        nMode = BATTLE;
    }
    else if ((LOW == bBumper1) && (LOW == bBumper2)) {
        nMode = BATTLE;
    }
    else if ((nDistance >= 50) && (nDistance < 100)) {
        nMode = AUTONOMOUS;
    } else {
        nMode = SERIALCMD;
    }

#ifdef HASOLED
    // Betriebsmodus auf dem OLED Display anzeigen
    // Show current mode of operation on the display
    showMode();
#endif

// --- BEGIN SETUP MODE ---


// --- BEGIN SETUP WS2811 ---

    pinMode(WS2811_PIN, OUTPUT);

    // LEDS initialisieren
    // initialise LEDs
    FastLED.addLeds<WS2811, WS2811_PIN, RGB>(leds, NUM_LEDS);

    // alle LEDs ausschalten
    // switch off all LEDs
    
    // fill_solid(leds, NUM_LEDS, CRGB::Black);

    // verschiedene Farben einstellen
    // set different colors
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

    // Helligkeit
    // brightness
    FastLED.setBrightness(50);

    // LEDs einschalten
    // switch on LEDs
    FastLED.show();

    // alle LEDs nacheinander ausschalten
    // switch the LEDs off one after the other 
    delay(500);
    for (int i=0; i<NUM_LEDS; i++) {
        leds[i] = CRGB::Black;
        FastLED.show();
        delay(50);
    }

    // Betriebsmodus mit Farben anzeigen
    // show mode of operation by different LED colors
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
            // LED Farbe abhängig von der Joystickposition setzen
            // set the LED color depending on the joystick position
            int nJoyHPos = analogRead(JOY_HORIZONTAL_PIN);
            int nJoyColorValue = map(nJoyHPos, 0, 1023, 0, 255);

            // feste LED Farbe einstellen
            // set a fixed color
            // leds[0] = CRGB::Blue;
            // leds[1] = CRGB::Blue;

            // Farben für jeden Kanal (RGB) einzeln einstellen
            // set the colors for each channel (RGB) separately
            leds[0].red = nJoyColorValue;
            leds[0].green = 255 - nJoyColorValue;
            leds[0].blue =  150;

            // Farben für alle Kanäle gleichzeitig setzen
            // set the colors for all channels at the same time
            leds[1] = CRGB( nJoyColorValue, 255 - nJoyColorValue, 150);

            FastLED.show();
    }

// --- END SETUP WS2811 ---


// --- BEGIN SETUP JOYSTICK ---

    // Das Setzen der Pins für den Joystick auf den Default INPUT
    // stört die Ansteuerung der Motoren, daher auf Default lassen.

	// Setting the joystick pins disturbs the motor control. Therefore
	// just use the default (although that should be INPUT as well).
	
    // pinMode(joyVert, INPUT);
    // pinMode(joyHorz, INPUT);

// --- END SETUP JOYSTICK ---

}

void loop() {

// --- BEGIN LOOP MOTOR ---

    // Handlung entsprechend dem Betriebsmodus ausführen
	  // act according to the mode of operation
	
    // nMode = SERIALCMD;

    switch (nMode) {
        case AUTONOMOUS:
            doSerialCommand();
            avoidObstacles();
            break;

        case BATTLE:
            doSerialCommand();
            doBattle();
            break;

        case SERIALCMD:
            doSerialCommand();
            break;

        case MANUAL:
            default:
            doSerialCommand();
            manualControl();
    }

// --- END LOOP MOTOR ---


// --- BEGIN LOOP SERVO ---

    // Servo 1 hin und her bewegen
    // move servo 1 back and forth
    moveServoBackForth();

// --- END LOOP SERVO ---


}


/*
  serialEvent() wird immer dann aufgerufen, wenn neue Daten über die serielle 
  Schnittstelle eintreffen. Der Einsatz von delay() in der loop() kann den
  Empfang verzögern. 
  
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.

  See https://www.arduino.cc/en/Tutorial/SerialEvent
*/
void serialEvent() {
  while (Serial.available()) {
    
    // Zeichen von der seriellen Schnittstelle lesen
    // read characters from the serial interface
    char chrSerialInput = (char)Serial.read();
    
    // ankommende Zeichen zu einem String zusammenfügen
    // collect the incoming characters
    strSerialInput += chrSerialInput;

    // ein Zeilenumbruch (LF) oder Zeilenrücklauf (CR) beendet den String.
    // bSerialComplete wird in der loop() ausgewertet
    
    // receiving CR or LF marks the command as complete
    // bSerialComplete is used in the loop()
    
    // if ((chrSerialInput == '\n') || (chrSerialInput == '\r')) {
    if (chrSerialInput == '\n') {
        bSerialComplete = true;
    }

  }
}
