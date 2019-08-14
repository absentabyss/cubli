#include <Wire.h>
#include <math.h>

// ACCEL/GYRO
long accelX , accelY , accelZ; // Output del accel
float gForceX , gForceY , gForceZ; // Accel en Gs

long gyroX , gyroY , gyroZ; // Output del gyro
float rotX , rotY , rotZ; // Gyro en grados/s gyroThreshold es para que el ruido del gyro no accione los motores. Creo que no importa igual

float xOrient , yOrient , zOrient , deltaY; // Orientacion. Los deltaX es xOrient - 90
int deltaLim = 30; // Este es el máximo delta en el cuál el motor va a hacer fuerza

long accelOffsetX = 0 , accelOffsetY = 0 , accelOffsetZ = 0 , rotOffsetX = 0 , rotOffsetY = 0 , rotOffsetZ = 0; // Para tarar el accel/gyro

// LED
int ledPin = 13;

// MOTOR
// Pin del motor
#define motorY 9 // Tienen que ser los pines 3, 5, 6, 9, 10 u 11 que pueden mandar PWM
#define x1 3
#define x2 2

int motorYSpeed = 40;

// Lo primero que hace ardu
void setup() {
  pinMode(motorY , OUTPUT);
  pinMode(ledPin , OUTPUT);
  Serial.begin(9600); // Para que ardu pueda printear a la compu
}

// Lo que hace todo el tiempo ardu
void loop() {
}

void serialEvent() {
	motorYSpeed = Serial.read() - 97;
	analogWrite(motorY , motorYSpeed);
  digitalWrite(ledPin , HIGH);
  delay(1000);
  digitalWrite(ledPin , LOW);
}
