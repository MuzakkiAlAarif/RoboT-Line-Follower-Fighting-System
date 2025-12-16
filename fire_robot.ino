#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ===== FORWARD DECLARATION =====
void mqttCallback(char* topic, byte* payload, unsigned int length);
void connectMQTT();

// =================================================
// PIN SENSOR LINE
// =================================================
#define S1 34  
#define S2 35
#define S3 32
#define S4 33
#define S5 25

// =================================================
// PIN MOTOR
// =================================================
#define IN1 14
#define IN2 27
#define ENA 13
#define IN3 26
#define IN4 12
#define ENB 23

// =================================================
// PIN FLAME & SERVO
// =================================================
#define FLAME_PIN 39
#define SERVO_RADAR_PIN 4
#define SERVO_LOCK_PIN 5
#define RELAY_PIN 18

Servo servoRadar;
Servo servoLock;

// ================= MQTT =================
const char* ssid = "NAMA_WIFI";
const char* password = "PASSWORD_WIFI";
const char* mqtt_server = "IP_LAPTOP";

WiFiClient espClient;
PubSubClient client(espClient);

bool cvFireAllowed = false;
bool fireJustExtinguished = false;

// =================================================
// PID VARIABLE
// =================================================
float Kp = 18;
float Ki = 0;
float Kd = 6;

float error = 0;
float filteredError = 0;
float lastError = 0;
float integral = 0;

// =================================================
// SPEED VARIABLE
// =================================================
int baseSpeed = 180;
int baseSpeedFast = 220;
int baseSpeedSlow = 140;
int targetBaseSpeed = 180;
int maxSpeed  = 350;

float alpha = 0.6;

// =================================================
// FIRE SYSTEM VARIABLE
// =================================================
bool fireDetected = false;
int radarAngle = 90;
int radarStep  = 3;

// =================================================
// SETUP
// =================================================
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);
  connectMQTT();

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  servoRadar.attach(SERVO_RADAR_PIN);
  servoLock.attach(SERVO_LOCK_PIN);

  servoRadar.write(90);
  servoLock.write(90);
}

// =================================================
// MOTOR STOP
// =================================================
void motorStop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// =================================================
// MOTOR DRIVE
// =================================================
void motorDrive(int left_us, int right_us) {
  left_us  = constrain(left_us,  0, maxSpeed);
  right_us = constrain(right_us, 0, maxSpeed);

  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
  delayMicroseconds(min(left_us, right_us));

  if (left_us > right_us) {
    digitalWrite(ENB, LOW);
    delayMicroseconds(left_us - right_us);
  } else if (right_us > left_us) {
    digitalWrite(ENA, LOW);
    delayMicroseconds(right_us - left_us);
  }

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

// =================================================
// LOOP
// =================================================
void loop() {

  if (!client.connected()) connectMQTT();
  client.loop();

  // ===== JIKA API BARU PADAM → PUTAR 180 =====
  if (fireJustExtinguished) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    motorDrive(200, 200);
    delay(900);
    motorStop();

    fireJustExtinguished = false;
  }

  // ===== JIKA CV TIDAK IZINKAN → STOP =====
  if (!cvFireAllowed) {
    motorStop();
    return;
  }

  // ===== LINE FOLLOWER =====
  error = readError();
  filteredError = alpha * filteredError + (1 - alpha) * error;

  float P = filteredError;
  float D = filteredError - lastError;
  lastError = filteredError;

  float output = Kp * P + Kd * D;

  int leftPWM  = baseSpeed - output * 35;
  int rightPWM = baseSpeed + output * 35;

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  motorDrive(leftPWM, rightPWM);
}

// =================================================
// MQTT CALLBACK
// =================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (msg == "ON") {
    cvFireAllowed = true;
    fireJustExtinguished = false;
    Serial.println("📡 CV FIRE ON");
  }

  if (msg == "OFF") {
    cvFireAllowed = false;
    fireJustExtinguished = true;
    Serial.println("📡 CV FIRE OFF");
  }
}

// =================================================
// CONNECT MQTT
// =================================================
void connectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32_FIREBOT")) {
      client.subscribe("fire/detect");
    } else {
      delay(2000);
    }
  }
}
