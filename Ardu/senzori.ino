#include <math.h>
#include <Adafruit_BNO08x.h>

// Configurația pentru motor și LIDAR (TFmini Plus)
#define DIR_PIN_Z 14       // Direcție pentru motorul de pe axa Z
#define STEP_PIN_Z 13      // Pas pentru motorul de pe axa Z
#define TFMINI_RX 5        // RX pentru TFmini Plus
#define TFMINI_TX 8        // TX pentru TFmini Plus

// Configurația pentru IMU (BNO08x)
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1

// Configurația motorului și a LIDAR-ului
const int stepsPerRevolution = 200;
const int motorRPM = 200;
const unsigned long sensorReadInterval = 10;
const unsigned long publishInterval = 10;  // Interval de publicare
const float degreesPerStep = 360.0 / stepsPerRevolution;

// Variabile pentru motor, LIDAR și IMU
volatile int stepCount = 0;
unsigned long previousMillisSensor = 10;
unsigned long previousMillisPublish = 10;
float currentAngle = 0.0;
int lastDistance = -1;

// Obiecte pentru IMU și motor
hw_timer_t *motorTimer = NULL;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Funcția de întrerupere pentru motor
void IRAM_ATTR motorStepISR() {
  static bool stepState = false;
  digitalWrite(STEP_PIN_Z, stepState);
  stepState = !stepState;

  if (!stepState) {
    stepCount++;
    if (stepCount >= stepsPerRevolution) {
      stepCount = 0;
    }
  }
}

// Citirea distanței de la TFmini Plus
int readDistanceTFmini() {
  int distance = -1;
  if (Serial1.available() >= 9) {
    if (Serial1.read() == 0x59 && Serial1.read() == 0x59) {
      int lowByte = Serial1.read();
      int highByte = Serial1.read();
      distance = (highByte << 8) + lowByte;
      for (int i = 0; i < 5; i++) Serial1.read(); // Golește restul bufferului
    }
  }
  return distance;
}

// Calculul coordonatelor X, Y și publicarea împreună cu quaternionii
void calculateAndPublishPoint(int distance, float real, float i, float j, float k) {
  currentAngle = (stepCount * degreesPerStep);
  float radians = currentAngle * (M_PI / 180.0);
  float x = distance * cos(radians);
  float y = distance * sin(radians);

  // Publicare date combinate
  Serial.print("data: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.print(distance);
  Serial.print(", ");
  Serial.print(real);
  Serial.print(", ");
  Serial.print(i);
  Serial.print(", ");
  Serial.print(j);
  Serial.print(", ");
  Serial.println(k);
}

// Inițializare rapoarte pentru IMU
void setReports() {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}

void setup() {
  // Configurație pentru motor
  pinMode(DIR_PIN_Z, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);
  digitalWrite(DIR_PIN_Z, HIGH);

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, TFMINI_RX, TFMINI_TX);

  // Configurație pentru timerul motorului
  motorTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(motorTimer, &motorStepISR, true);
  timerAlarmWrite(motorTimer, (1000000 * 60) / (motorRPM * stepsPerRevolution), true);
  timerAlarmEnable(motorTimer);

  // Inițializare IMU
  Serial.println("Adafruit BNO08x test!");
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }

  setReports();
  Serial.println("Reading events");
  delay(100);
}

void loop() {
  // Citire TFmini Plus
  unsigned long currentMillisSensor = millis();
  if (currentMillisSensor - previousMillisSensor >= sensorReadInterval) {
    previousMillisSensor = currentMillisSensor;
    int distance = readDistanceTFmini();
    if (distance != -1) {
      lastDistance = distance;
    }
  }

  // Citire IMU
  float real = 0.0, i = 0.0, j = 0.0, k = 0.0;
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      real = sensorValue.un.gameRotationVector.real;
      i = sensorValue.un.gameRotationVector.i;
      j = sensorValue.un.gameRotationVector.j;
      k = sensorValue.un.gameRotationVector.k;

      // Afișare imediată a valorilor quaternion
      Serial.print("data: ");
      Serial.print(currentAngle);
      Serial.print(", ");
      Serial.print(lastDistance);
      Serial.print(", ");
      Serial.print(real);
      Serial.print(", ");
      Serial.print(i);
      Serial.print(", ");
      Serial.print(j);
      Serial.print(", ");
      Serial.println(k);
    }
  }

  // Publicare date la intervalul setat
  unsigned long currentMillisPublish = millis();
  if (currentMillisPublish - previousMillisPublish >= publishInterval) {
    previousMillisPublish = currentMillisPublish;
    if (lastDistance != -1) {
      calculateAndPublishPoint(lastDistance, real, i, j, k);
    }
  }
}