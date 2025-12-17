#include <ESP32Servo.h>
#include <WiFi.h>
#include <PubSubClient.h>

/* ================= WiFi & MQTT CONFIG ================= */
const char* ssid = "seipa";           // Ganti dengan WiFi Anda
const char* password = "00000001";   // Ganti dengan password WiFi
const char* mqtt_server = "10.22.41.17"; // FIX: hapus spasi depan
const int mqtt_port = 1883;

const char* topic_fire_detected = "robot/fire/detected";  // Subscribe - OpenCV kirim sinyal
const char* topic_status = "robot/status";                 // Publish - status robot

WiFiClient espClient;
PubSubClient client(espClient);

/* ================= PIN LINE SENSOR ================= */
#define S1 34
#define S2 35
#define S3 32
#define S4 33
#define S5 25

/* ================= MOTOR DRIVER ================= */
#define IN1 14
#define IN2 27
#define ENA 13
#define IN3 26
#define IN4 12
#define ENB 23

/* ================= FIRE SYSTEM ================= */
#define FLAME_PIN 39       // LOW = flame detected
#define RELAY_PIN 18

/* ================= SERVO (OPTIONAL) ================= */
#define SERVO_RADAR_PIN 4
#define SERVO_LOCK_PIN 5

Servo servoRadar;
Servo servoLock;

/* ================= PID LINE FOLLOWER ================= */
float Kp = 10;    // Kurangi Kp untuk smooth
float Kd = 4;     // Kurangi Kd
float lastError = 0;
float filteredError = 0;
float alpha = 0.5;

/* ================= SPEED ================= */
int baseSpeed = 100;   // Kecepatan sedang
int maxSpeed = 120;    // Max speed lebih rendah

/* ================= STATE MACHINE ================= */
enum RobotState {
  IDLE,              // Menunggu sinyal MQTT
  MOVING_TO_FIRE,    // Bergerak menuju api mengikuti line
  EXTINGUISHING,     // Memadamkan api (pompa aktif)
  TURN_BACK_FROM_FIRE,   // (NEW) Putar 180 derajat setelah padam
  RETURNING,         // Kembali ke posisi awal
  FINAL_TURN_AT_START,   // (NEW) Putar 180 derajat saat sampai start
  COMPLETED          // Misi selesai
};

RobotState currentState = IDLE;

/* ================= FLAGS & TIMERS ================= */
bool missionActive = false;
unsigned long extinguishStartTime = 0;
const unsigned long extinguishDuration = 5000; // Pompa nyala 5 detik

/* ================================================= */
void setup() {
  Serial.begin(115200);

  // Sensor Setup
  pinMode(S1, INPUT); pinMode(S2, INPUT); pinMode(S3, INPUT);
  pinMode(S4, INPUT); pinMode(S5, INPUT);

  // Motor Setup
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  // Fire System Setup
  pinMode(FLAME_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Servo Setup
  servoRadar.attach(SERVO_RADAR_PIN);
  servoLock.attach(SERVO_LOCK_PIN);
  servoRadar.write(90);
  servoLock.write(90);

  // WiFi & MQTT Setup
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);

  Serial.println("=================================");
  Serial.println("🤖 FIRE FIGHTING ROBOT READY");
  Serial.println("⏸️  STATE: IDLE - Waiting for fire signal...");
  Serial.println("=================================");
}

/* ================================================= */
void loop() {
  // Maintain MQTT connection
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  // State Machine
  switch (currentState) {

    /* ========== IDLE - Menunggu sinyal MQTT ========== */
    case IDLE:
      motorStop();
      // Menunggu callback MQTT mengubah state
      break;

    /* ========== MOVING_TO_FIRE - Menuju api ========== */
    case MOVING_TO_FIRE:
      // Cek flame sensor dulu
      if (digitalRead(FLAME_PIN) == LOW) {
        Serial.println("🔥 FLAME SENSOR TRIGGERED!");
        Serial.println("🛑 STOPPING - Preparing to extinguish...");
        currentState = EXTINGUISHING;
        extinguishStartTime = millis();
        motorStop();
        delay(500); // Brief stop
        digitalWrite(RELAY_PIN, HIGH); // Pompa ON
        client.publish(topic_status, "EXTINGUISHING");
        break;
      }

      // Jika belum detect api, terus ikuti line
      lineFollower();
      break;

    /* ========== EXTINGUISHING - Memadamkan api ========== */
    case EXTINGUISHING:
      motorStop(); // Tetap diam

      // Pompa nyala selama durasi yang ditentukan
      if (millis() - extinguishStartTime >= extinguishDuration) {
        digitalWrite(RELAY_PIN, LOW); // Pompa OFF
        Serial.println("💧 PUMP OFF");
        Serial.println("✅ Fire extinguished! Starting return maneuver...");

        // (CHANGE) setelah padam, putar 180 dulu lewat state khusus
        currentState = TURN_BACK_FROM_FIRE;
        client.publish(topic_status, "TURN_BACK_FROM_FIRE");
      }
      break;

    /* ========== TURN_BACK_FROM_FIRE - Putar 180 setelah padam ========== */
    case TURN_BACK_FROM_FIRE:
      motorStop();
      delay(200);

      executeReturnManeuver(); // ini hanya muter 180 sesuai fungsi kamu

      // Setelah muter 180, baru balik line follower
      currentState = RETURNING;
      client.publish(topic_status, "RETURNING");
      break;

    /* ========== RETURNING - Kembali ke start ========== */
    case RETURNING:
      // Cek apakah sudah sampai di posisi start (MARKER START)
      if (isAtStartPosition()) {
        motorStop();
        Serial.println("🏁 ARRIVED AT START POSITION");
        Serial.println("🔄 Final 180° turn at start...");

        currentState = FINAL_TURN_AT_START;
        client.publish(topic_status, "FINAL_TURN_AT_START");
        break;
      }

      // Terus ikuti line kembali ke start
      lineFollower();
      break;

    /* ========== FINAL_TURN_AT_START - Putar 180 di start ========== */
    case FINAL_TURN_AT_START:
      motorStop();
      delay(200);

      rotateInPlace(100, 800); // 180 derajat kedua (kalibrasi sama seperti sebelumnya)
      delay(200);

      Serial.println("✅ MISSION COMPLETED");
      currentState = COMPLETED;
      client.publish(topic_status, "COMPLETED");
      missionActive = false;
      break;

    /* ========== COMPLETED - Misi selesai ========== */
    case COMPLETED:
      motorStop();
      delay(500); // Pause sejenak

      // Reset ke IDLE untuk menunggu misi baru
      Serial.println("🔄 Resetting to IDLE state...");
      currentState = IDLE;
      client.publish(topic_status, "IDLE");
      break;
  }
}

/* ================= WiFi SETUP ================= */
void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("📡 Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("✅ WiFi connected!");
  Serial.print("📍 IP address: ");
  Serial.println(WiFi.localIP());
}

/* ================= MQTT RECONNECT ================= */
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("🔌 Connecting to MQTT...");

    String clientId = "ESP32FireRobot-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println(" connected!");
      client.subscribe(topic_fire_detected);
      client.publish(topic_status, "CONNECTED");
      Serial.println("📩 Subscribed to: " + String(topic_fire_detected));
    } else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" - retrying in 5 seconds");
      delay(500);
    }
  }
}

/* ================= MQTT CALLBACK ================= */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.println("📨 MQTT Message received on topic: " + String(topic));
  Serial.println("📝 Message: " + message);

  // Jika terima sinyal fire detected dari OpenCV
  if (String(topic) == topic_fire_detected) {
    if (message == "1" || message == "ON" || message == "FIRE_DETECTED") {

      // Hanya mulai misi jika state IDLE atau COMPLETED
      if (currentState == IDLE || currentState == COMPLETED) {
        Serial.println("🚨 FIRE ALERT RECEIVED FROM OPENCV!");
        Serial.println("🏃 Starting mission - Moving to fire...");

        currentState = MOVING_TO_FIRE;
        missionActive = true;
        client.publish(topic_status, "MOVING_TO_FIRE");
      } else {
        Serial.println("⚠️  Mission already in progress - ignoring signal");
      }
    }
  }
}

/* ================= LINE FOLLOWER ================= */
void lineFollower() {
  int error = readLineError();
  filteredError = alpha * filteredError + (1 - alpha) * error;
  float derivative = filteredError - lastError;
  lastError = filteredError;

  float output = Kp * filteredError + Kd * derivative;

  // Faktor pengali lebih kecil untuk gerakan smooth
  int leftSpeed  = baseSpeed - output * 20;
  int rightSpeed = baseSpeed + output * 20;

  driveForward(leftSpeed, rightSpeed);
}

/* ================= SENSOR ERROR READING ================= */
int readLineError() {
  // Baca semua sensor (LOW = hitam/garis, HIGH = putih)
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  // Debug sensor reading (hapus setelah testing)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Sensors: ");
    Serial.print(s1); Serial.print(" ");
    Serial.print(s2); Serial.print(" ");
    Serial.print(s3); Serial.print(" ");
    Serial.print(s4); Serial.print(" ");
    Serial.println(s5);
    lastPrint = millis();
  }

  // TCRT: LOW = detect garis hitam, HIGH = putih
  // Cek kombinasi sensor untuk posisi garis

  // Tengah - garis di sensor 3
  if (s3 == LOW) return 0;

  // Kiri - garis di sensor 2
  if (s2 == LOW && s3 == HIGH) return -1;

  // Kanan - garis di sensor 4
  if (s4 == LOW && s3 == HIGH) return 1;

  // Kiri banget - garis di sensor 1
  if (s1 == LOW) return -2;

  // Kanan banget - garis di sensor 5
  if (s5 == LOW) return 2;

  // Semua putih (lost line) - maintain last error
  if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) {
    return (lastError > 0) ? 2 : -2; // Belok ke arah terakhir
  }

  return 0; // Default tengah
}

/* ================= MOTOR CONTROL ================= */
void motorStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void driveForward(int left, int right) {
  left = constrain(left, 0, maxSpeed);
  right = constrain(right, 0, maxSpeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, left);
  analogWrite(ENB, right);
}

void driveBackward(int speed) {
  speed = constrain(speed, 0, maxSpeed);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void rotateInPlace(int speed, int duration) {
  speed = constrain(speed, 0, maxSpeed);

  // Rotate kiri (left forward, right backward)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);

  delay(duration);
  motorStop();
}

/* ================= RETURN MANEUVER ================= */
void executeReturnManeuver() {
  Serial.println("🔄 Executing 180° turn maneuver...");

  // 2. Rotate 180 derajat
  Serial.println("🔄 Step 2: Rotating 180°...");
  rotateInPlace(100, 800); // Kecepatan sedang untuk rotasi
  delay(300);

  Serial.println("➡️  Step 3: Following line back to start...");
  // State RETURNING akan handle line following
}

/* ================= CHECK START POSITION ================= */
bool isAtStartPosition() {
  // (CHANGE) Lebih aman: marker start = semua sensor LOW (hitam) beberapa kali berturut-turut.
  // Kamu bisa bikin tape hitam melintang / start box hitam.

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);

  static int blackCounter = 0;

  if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
    blackCounter++;
    if (blackCounter > 8) { // Konfirmasi beberapa kali
      blackCounter = 0;
      return true;
    }
  } else {
    blackCounter = 0;
  }

  return false;
}

/* ================= DEBUG INFO ================= */
void printDebugInfo() {
  Serial.print("State: ");
  switch(currentState) {
    case IDLE: Serial.print("IDLE"); break;
    case MOVING_TO_FIRE: Serial.print("MOVING_TO_FIRE"); break;
    case EXTINGUISHING: Serial.print("EXTINGUISHING"); break;
    case TURN_BACK_FROM_FIRE: Serial.print("TURN_BACK_FROM_FIRE"); break;
    case RETURNING: Serial.print("RETURNING"); break;
    case FINAL_TURN_AT_START: Serial.print("FINAL_TURN_AT_START"); break;
    case COMPLETED: Serial.print("COMPLETED"); break;
  }

  Serial.print(" | Flame: ");
  Serial.print(digitalRead(FLAME_PIN) == LOW ? "DETECTED" : "NONE");

  Serial.print(" | Line: ");
  Serial.print(readLineError());

  Serial.println();
}
