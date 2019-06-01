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
#define motorY 5 // Tienen que ser los pines 3, 5, 6, 9, 10 u 11 que pueden mandar PWM
#define x1 3
#define x2 2

int motorYSpeed = 0;

// Lo primero que hace ardu
void setup() {
  pinMode(ledPin , OUTPUT); // Le dice a ardu que el pin ledPin es para escribir información
  pinMode(motorY , OUTPUT);
  pinMode(x1 , OUTPUT);
  pinMode(x2 , OUTPUT);
  Serial.begin(9600); // Para que ardu pueda printear a la compu
  Wire.begin(); // Esto creo que inicia la conexión I2C
  setupMPU();
  recordAccelRegisters(); // Los offsets definen a los valores iniciales de orientación y rotación como 0
  accelOffsetX = accelX;
  accelOffsetY = accelY;
  accelOffsetZ = accelZ - 16384;
  recordGyroRegisters();
  rotOffsetX = gyroX;
  rotOffsetY = gyroY;
  rotOffsetZ = gyroZ;
}

// Lo que hace todo el tiempo ardu
void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  processOrient();
  printData();
}

void setupMPU(){ // Settea el modo del accel/gyro
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() { // Pide data al accel
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){ // Conversión del output del accel a Gs
  gForceX = (accelX - accelOffsetX) / 16384.0;
  gForceY = (accelY - accelOffsetY) / 16384.0; 
  gForceZ = (accelZ - accelOffsetZ) / 16384.0;
}

void recordGyroRegisters() { // Pide data al gyro
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() { // Conversión del output del gyro a grados/s
  rotX = (gyroX - rotOffsetX) / 131.0;
  rotY = (gyroY - rotOffsetY) / 131.0; 
  rotZ = (gyroZ - rotOffsetZ) / 131.0;
}

void processOrient(){ // Usa los datos de gForce para saber la orientación de accel/gyro
  float Force = sqrt(pow(gForceX , 2) + pow(gForceY , 2) + pow(gForceZ , 2)); // Módulo del vector (gForceX , gForceY , gForceZ)
  if(Force == (float)0){ // Evitar división por 0
    Force = (float)100;
  }
  xOrient = 180.0 * acos(gForceX / Force) / M_PI; // Conversiones a grados normales
  yOrient = 180.0 * acos(gForceY / Force) / M_PI;
  zOrient = 180.0 * acos(gForceZ / Force) / M_PI;
  if(fabs(90.0 - xOrient) < 1 && fabs(90.0 - yOrient) < 1 && fabs(zOrient) < 1){ // Prenda luz cuando está nivelado
    digitalWrite(ledPin , HIGH);
  }else{
    digitalWrite(ledPin , LOW);
  }
  deltaY = yOrient - 90.0;
  if(fabs(deltaY) <= deltaLim){
    if(deltaY >= 0){ // Controla la dirección del puente H
      digitalWrite(x1 , HIGH);
      digitalWrite(x2 , LOW);
    }else{
      digitalWrite(x1 , LOW);
      digitalWrite(x2 , HIGH);
    }
    motorYSpeed = map(fabs(deltaY) , 0 , deltaLim , 0 , 255); // Transforma la rotación a fuerza de motor (No va a funcionar, pero es para tener algo)
    analogWrite(motorY , motorYSpeed);
  }else{
    digitalWrite(x1 , LOW);
    digitalWrite(x2 , LOW);
    analogWrite(motorY , 0);
  }
}

void printData() { // Printea toda la data a la compu
  Serial.print("Orientation X = ");
  Serial.print(xOrient);
  Serial.print(" Y = ");
  Serial.print(yOrient);
  Serial.print(" Z = ");
  Serial.print(zOrient);
  Serial.print("  Rotation X = ");
  Serial.print(rotX);
  Serial.print(" Y = ");
  Serial.print(rotY);
  Serial.print(" Z = ");
  Serial.println(rotZ);
}
