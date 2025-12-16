# 🔥 Fire-Fighting Line Follower Robot

<img width="1327" height="623" alt="image" src="https://github.com/user-attachments/assets/82065ff2-8237-48d8-b714-4dea31b81a23" />

Robot **Line Follower berbasis ESP32-WROOM** yang terintegrasi dengan **Computer Vision (kamera laptop)** menggunakan **MQTT** sebagai protokol komunikasi.

Robot **TIDAK akan bergerak** kecuali sistem Computer Vision mendeteksi api melalui kamera. Ketika api terdeteksi, robot akan mengikuti garis (line follower). Saat api padam, robot akan **berhenti dan berputar 180°**.

---

## 📌 Fitur Utama

* 🤖 Line Follower stabil berbasis PID
* 🔥 Deteksi api menggunakan **kamera + OpenCV (Computer Vision)**
* 📡 Komunikasi **ESP32 ↔ Laptop** menggunakan **MQTT**
* 🚒 Pompa air otomatis menggunakan **relay**
* 🎯 Servo radar & servo lock untuk arah api
* 🔄 Robot berputar 180° saat api padam

---

## 🧠 Arsitektur Sistem

```text
[KAMERA LAPTOP]
      │
      │  (OpenCV - Fire Detection)
      │
[MQTT BROKER - Laptop]
      │  Topic: fire/detect
      │  Payload: ON / OFF
      │
[ESP32 WROOM]
      │
      ├─ Line Follower Aktif (ON)
      ├─ Robot Stop + Putar 180° (OFF)
      └─ Relay Pompa Air
```

---

## 🧰 Hardware yang Digunakan

| Komponen                | Keterangan           |
| ----------------------- | -------------------- |
| ESP32 WROOM             | Mikrokontroler utama |
| Kamera Laptop / USB Cam | Deteksi api          |
| 5x Sensor Line          | Sensor garis         |
| Driver Motor            | L298N / BTS          |
| 2x Servo Motor          | Radar & Lock         |
| Relay Module            | Kontrol pompa        |
| Pompa Air               | Pemadam api          |
| Laptop                  | CV + MQTT            |

---

## 💻 Software yang Digunakan

| Bagian          | Software                      |
| --------------- | ----------------------------- |
| ESP32           | **Arduino IDE**               |
| Computer Vision | **Python + OpenCV (VS Code)** |
| MQTT Broker     | **Mosquitto**                 |

---

## 📂 Struktur Folder Project

```text
fire-fighting-line-follower/
│
├── esp32/
│   └── fire_robot.ino
│
├── vision/
│   └── fire_detector.py
│
├── docs/
│   └── wiring_diagram.png
│
└── README.md
```

---

## 🚀 Step-by-Step Instalasi

### 1️⃣ Setup MQTT Broker (Laptop)

Install Mosquitto:

```bash
https://mosquitto.org/download/
```

Jalankan broker:

```bash
mosquitto -v
```

---

### 2️⃣ Setup Computer Vision (Laptop)

Install Python library:

```bash
pip install opencv-python numpy paho-mqtt
```

Jalankan program CV:

```bash
python fire_detector.py
```

📌 Program ini akan:

* Mengakses kamera
* Mendeteksi warna api
* Mengirim MQTT `ON` / `OFF`

---

### 3️⃣ Setup ESP32 (Robot)

1. Buka **Arduino IDE**
2. Pilih Board: `ESP32 Dev Module`
3. Install library:

   * ESP32Servo
   * PubSubClient
4. Upload file:

```text
esp32/fire_robot.ino
```

---

## 📡 MQTT Protocol

| Parameter | Value         |
| --------- | ------------- |
| Broker    | Laptop IP     |
| Port      | 1883          |
| Topic     | `fire/detect` |
| Payload   | `ON`, `OFF`   |

---

## 🔄 Alur Kerja Robot

1. Kamera mendeteksi api
2. Laptop kirim MQTT `ON`
3. Robot line follower aktif
4. Api padam → MQTT `OFF`
5. Robot berhenti & putar 180°
6. Robot standby

---

## ⚠️ Catatan Penting

* ESP32 & Laptop **HARUS satu jaringan WiFi**
* Kamera harus stabil (hindari cahaya berlebih)
* Delay putar 180° perlu dikalibrasi sesuai chassis

---

## 📈 Pengembangan Selanjutnya

* 🔥 Fire detection berbasis YOLO / AI
* 🎯 Servo mengikuti arah api dari kamera
* 🧠 Mapping ruangan
* 📱 Monitoring via dashboard

---

## 👨‍💻 Author

**Fire-Fighting Line Follower Robot Project**
ESP32 + Computer Vision + MQTT

---

> "Integrating Robotics, IoT, and Computer Vision for Intelligent Fire Response" 🔥🤖
