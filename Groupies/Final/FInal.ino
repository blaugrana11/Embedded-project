#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
#include <Servo.h>

// Moteurs
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotorLEFT = AFMS.getMotor(3);
Adafruit_DCMotor *myMotorRIGHT = AFMS.getMotor(4);

// Capteurs IR
#define IR_LEFT A0
#define IR_RIGHT A1

// Capteur ultrason
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 200
NewPing DistanceSensor(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// Servo
Servo servo;
#define SERVO_PIN 3
int spoint = 90; // Pos du servo

// PID
float kp = 1.5, ki = 0.0, kd = 0.5;
float previousError = 0, integral = 0;
int baseSpeed = 50;

// detect d'obstacles
const int obstacleThreshold = 15;

int L = 0, R = 0, distance = 0;

//------------------------------
const int buttonPin = 8;       // Pin pour le bouton poussoir
bool turnRight = false;        // Variable pour gérer la direction de rotation

// Bouton poussoir d'arrêt d'urgence
const int emergencyStopButtonPin = 7; // Nouveau pin pour le bouton d'arrêt d'urgence
bool emergencyStopActive = false;     // Variable pour suivre l'état du bouton d'arrêt d'urgence

//------------------------------


void setup() {
    Serial.begin(9600);
    AFMS.begin();
//------------------------------
    pinMode(buttonPin, INPUT_PULLUP);
    // Vérifier si le bouton est appuyé au démarrage
    if (digitalRead(buttonPin) == LOW) {
        turnRight = true;  // Le robot tournera à droite
        Serial.println("Mode: Tourner à DROITE activé !");
    } else {
        turnRight = false; // Par défaut : tourner à gauche
        Serial.println("Mode: Tourner à GAUCHE activé !");
    }

    // Configuration des moteurs
    myMotorLEFT->setSpeed(0);
    myMotorRIGHT->setSpeed(0);

    // Configuration du bouton d'arrêt d'urgence
    pinMode(emergencyStopButtonPin, INPUT_PULLUP);  // Le bouton d'arrêt d'urgence est en INPUT_PULLUP
//------------------------------

    pinMode(A0, INPUT);
    pinMode(A1, INPUT);

    servo.attach(SERVO_PIN);
    servo.write(spoint); // Position initiale (centrale)
    delay(500);
}

void loop() {
    //------------------------------
    if (digitalRead(emergencyStopButtonPin) == LOW) {  // Si le bouton est appuyé
        emergencyStop();
    }


    int valueLeft = analogRead(IR_LEFT);    // Lecture du capteur gauche
    int valueRight = analogRead(IR_RIGHT);  // Lecture du capteur droit

    // Seuil pour détecter la ligne blanche
    int threshold = 800;

    // Détection de la discontinuité (zone 9)
    if (valueLeft < threshold && valueRight < threshold) {
        delay(100);  // Pause pour confirmer
        valueLeft = analogRead(IR_LEFT);
        valueRight = analogRead(IR_RIGHT);

        if (valueLeft < threshold && valueRight < threshold) {
            Serial.println("Discontinuité détectée - Entrée dans la zone 9 !");
            
            // Tourner selon le mode activé
            if (turnRight) {
                TurnRightIntoZone();
            } else {
                TurnLeftIntoZone();
            }

            // Avancer pendant 2 secondes
            forward();
            delay(2000);

            // Arrêter complètement le robot
            Stop();
            Serial.println("Arrêt complet - Zone 9 atteinte !");
            while (1);  // Boucle infinie pour arrêt complet
        }
    }
    //------------------------------
    distance = DistanceSensor.ping_cm();
    if (distance > 0 && distance <= obstacleThreshold) {
        Obstacle();
        return;
    }

    int valueLeft = analogRead(IR_LEFT);
    int valueRight = analogRead(IR_RIGHT);

    int error = valueRight - valueLeft;
    float correction = computePID(error);

    int speedMotorLEFT = constrain(baseSpeed - correction, 0, 50);
    int speedMotorRIGHT = constrain(baseSpeed + correction, 0, 50);

    myMotorLEFT->setSpeed(speedMotorLEFT);
    myMotorLEFT->run(FORWARD);

    myMotorRIGHT->setSpeed(speedMotorRIGHT);
    myMotorRIGHT->run(FORWARD);

    Serial.print("Distance: "); Serial.print(distance);
    Serial.print(" Error: "); Serial.print(error);
    Serial.print(" Correction: "); Serial.println(correction);

    delay(10); 
}

float computePID(float error) {
    integral += error;
    float derivative = error - previousError;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}

void Obstacle() {
    Stop();
    backward();
    delay(500);
    Stop();

    servo.write(45); // vers la gauche
    delay(800);
    L = DistanceSensor.ping_cm();

    servo.write(135); // vers la droite
    delay(800);
    R = DistanceSensor.ping_cm();

    servo.write(spoint); // Retour au centre
    delay(500);

    if (L < R) {
        right();
        delay(500);
        Stop();
        delay(200);
    } else if (L > R) {
        left();
        delay(500);
        Stop();
        delay(200);
    } else {
        forward();
    }
}

void forward() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(FORWARD);
}

void backward() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(BACKWARD);
}

void right() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(BACKWARD);
}

void left() {
    myMotorLEFT->setSpeed(baseSpeed);
    myMotorRIGHT->setSpeed(baseSpeed);
    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(FORWARD);
}

void Stop() {
    myMotorLEFT->setSpeed(0);
    myMotorRIGHT->setSpeed(0);
    myMotorLEFT->run(RELEASE);
    myMotorRIGHT->run(RELEASE);
}

//------------------------------
// Fonction pour tourner à gauche dans la zone
void TurnLeftIntoZone() {
    myMotorLEFT->setSpeed(baseSpeed / 2);  // Vitesse réduite à gauche
    myMotorRIGHT->setSpeed(baseSpeed);     // Vitesse normale à droite

    myMotorLEFT->run(BACKWARD);
    myMotorRIGHT->run(FORWARD);
    delay(500);  // Durée du virage (à ajuster)
    Stop();
    delay(200);
}

// Fonction pour tourner à droite dans la zone
void TurnRightIntoZone() {
    myMotorLEFT->setSpeed(baseSpeed);      // Vitesse normale à gauche
    myMotorRIGHT->setSpeed(baseSpeed / 2); // Vitesse réduite à droite

    myMotorLEFT->run(FORWARD);
    myMotorRIGHT->run(BACKWARD);
    delay(500);  // Durée du virage (à ajuster)
    Stop();
    delay(200);
}


void emergencyStop() {
    Serial.println("Arrêt d'urgence activé !");
    Stop();  // Arrêter immédiatement les moteurs
    while (1);  // Boucle infinie pour s'arrêter
}
//------------------------------