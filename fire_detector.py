import cv2
import numpy as np
import sys
import time
import winsound
import paho.mqtt.client as mqtt

# ================== CONFIG ==================
USE_MQTT = True
BROKER = "10.22.41.17"
PORT = 1883

# HARUS sama dengan .ino
TOPIC_FIRE = "robot/fire/detected"   # ESP32 subscribe ini
TOPIC_STATUS = "robot/status"        # ESP32 publish ini (monitor state)

PAYLOAD_ON  = "ON"
PAYLOAD_OFF = "OFF"

# --- HSV Fire Range (punya kamu) ---
lower_fire = np.array([0, 50, 180], dtype="uint8")
upper_fire = np.array([35, 255, 255], dtype="uint8")

# --- Fire decision ---
FIRE_AREA_THRESHOLD = 100
DEBOUNCE_ON_HITS = 2
DEBOUNCE_OFF_HITS = 3

# --- Beep ---
BEEP_FREQ = 2500
BEEP_MS = 100
BEEP_COOLDOWN = 0.25

# ================== MQTT STATE ==================
client = None
mqtt_ready = False

last_robot_status = "-"     # status terakhir dari ESP
mission_latched = False     # True = mission sedang jalan, jangan retrigger ON

# robot siap menerima start mission kalau status ini:
ROBOT_READY_STATES = {"IDLE", "COMPLETED"}

# robot dianggap sedang menjalankan mission kalau status ini:
ROBOT_BUSY_STATES = {
    "MOVING_TO_FIRE",
    "EXTINGUISHING",
    "TURN_BACK_FROM_FIRE",
    "RETURNING",
    "FINAL_TURN_AT_START"
}

def now_s():
    return time.time()

def safe_decode(b: bytes) -> str:
    try:
        return b.decode(errors="ignore").strip()
    except:
        return ""

# ================== MQTT CALLBACKS ==================
def on_connect(client, userdata, flags, rc):
    global mqtt_ready
    mqtt_ready = (rc == 0)
    if mqtt_ready:
        print(f"\n✅ MQTT: Connected to {BROKER}:{PORT}")
        client.subscribe(TOPIC_STATUS)
    else:
        print(f"\n❌ MQTT: Connect failed rc={rc}")

def on_disconnect(client, userdata, rc):
    global mqtt_ready
    mqtt_ready = False
    print(f"\n⚠️ MQTT: Disconnected rc={rc}")

def on_message(client, userdata, msg):
    global last_robot_status, mission_latched

    if msg.topic != TOPIC_STATUS:
        return

    status = safe_decode(msg.payload)
    if not status:
        return

    last_robot_status = status

    # Latch logic: kalau robot mulai mission -> latch ON
    if status in ROBOT_BUSY_STATES:
        mission_latched = True

    # Kalau robot kembali IDLE -> boleh trigger lagi
    if status in ROBOT_READY_STATES or status == "IDLE":
        mission_latched = False

# ================== MQTT HELPERS ==================
def mqtt_init():
    global client, mqtt_ready, USE_MQTT

    if not USE_MQTT:
        return

    try:
        # Callback API v1 agar kompatibel dengan banyak versi paho
        client = mqtt.Client(client_id="OpenCV-FireDetector", clean_session=True)
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        client.on_message = on_message

        # safety: kalau PC mati mendadak, publish OFF (tidak mengganggu ESP, hanya safety/log)
        client.will_set(TOPIC_FIRE, payload=PAYLOAD_OFF, qos=0, retain=False)

        client.connect(BROKER, PORT, 60)
        client.loop_start()

        # tunggu connect max 2 detik
        t0 = now_s()
        while not mqtt_ready and (now_s() - t0) < 2.0:
            time.sleep(0.05)

        if not mqtt_ready:
            raise RuntimeError("MQTT connect timeout (broker tidak respon / beda jaringan / firewall)")

    except Exception as e:
        print(f"⚠️ MQTT tidak tersedia: {e}")
        USE_MQTT = False
        mqtt_ready = False
        client = None

def mqtt_publish(payload: str) -> bool:
    global USE_MQTT, mqtt_ready, client

    if not USE_MQTT:
        return False

    if (not mqtt_ready) or (client is None):
        mqtt_init()
        if not mqtt_ready:
            return False

    try:
        info = client.publish(TOPIC_FIRE, payload, qos=0, retain=False)
        info.wait_for_publish(timeout=1.0)
        return True
    except Exception as e:
        print(f"\n⚠️ MQTT publish gagal: {e}")
        mqtt_ready = False
        return False

mqtt_init()

# ================== CAMERA ==================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Kamera tidak terdeteksi!")
    sys.exit()

print("SISTEM PENDETEKSI API + ALARM + MQTT (TERINTEGRASI DENGAN ESP32)")
print("Tekan 'd' untuk toggle mask debug, 'q' untuk keluar.")
print("-----------------------------------")

show_mask = False

# status publish terakhir (biar tidak spam)
fire_sent = "OFF"

# debounce counters
on_hits = 0
off_hits = 0

last_beep_t = 0.0

# optional: batasi publish ON minimal jeda (anti spam jika status robot belum kebaca)
MIN_ON_INTERVAL = 1.0
last_on_publish_t = 0.0

def robot_is_ready() -> bool:
    """
    Robot siap start mission kalau:
    - belum latch (mission belum jalan)
    - status robot IDLE/COMPLETED (kalau status belum terbaca '-', tetap boleh, tapi lebih aman tunggu IDLE)
    """
    if mission_latched:
        return False
    if last_robot_status in ROBOT_READY_STATES:
        return True
    return False

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (640, 480))

        blur = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, lower_fire, upper_fire)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        # fitur
        _, _, v = cv2.split(hsv)
        max_brightness = int(np.max(v))
        fire_area = int(cv2.countNonZero(mask))

        raw_fire = fire_area > FIRE_AREA_THRESHOLD

        # debounce
        if raw_fire:
            on_hits += 1
            off_hits = 0
        else:
            off_hits += 1
            on_hits = 0

        # hasil debounce
        fire_detected = False
        if on_hits >= DEBOUNCE_ON_HITS:
            fire_detected = True
        if off_hits >= DEBOUNCE_OFF_HITS:
            fire_detected = False

        # beep
        if fire_detected:
            now = now_s()
            if (now - last_beep_t) >= BEEP_COOLDOWN:
                winsound.Beep(BEEP_FREQ, BEEP_MS)
                last_beep_t = now

        # ================== INTEGRASI LOGIC ==================
        # Kita cuma perlu kirim ON untuk memulai mission.
        # OFF tidak dipakai ESP untuk reset, tapi tetap kita kirim saat keluar program (safety).
        desired = "ON" if fire_detected else "OFF"

        # 1) Kalau api terdeteksi dan robot siap -> publish ON sekali
        if desired == "ON":
            can_send = robot_is_ready() and ((now_s() - last_on_publish_t) >= MIN_ON_INTERVAL)
            if can_send and fire_sent != "ON":
                ok = mqtt_publish(PAYLOAD_ON)
                fire_sent = "ON"
                last_on_publish_t = now_s()
                print(f"\n📡 PUBLISH {TOPIC_FIRE} = {PAYLOAD_ON} ({'OK' if ok else 'SKIP'})")
            # kalau robot belum siap / mission latched -> jangan publish ON lagi

        # 2) Kalau CV sudah OFF, kita boleh update fire_sent ke OFF untuk UI,
        #    tapi tidak perlu publish OFF berkali-kali (ESP tidak butuh).
        if desired == "OFF":
            fire_sent = "OFF"

        # bounding box
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > FIRE_AREA_THRESHOLD:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(frame, "API!", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        status_txt = "⚠️ KEBAKARAN! ⚠️" if fire_detected else "AMAN"
        ready_txt = "READY" if robot_is_ready() else "BUSY"

        sys.stdout.write(
            f"\r[MONITOR] Bright:{max_brightness:<3} | Area:{fire_area:<6} | CV:{status_txt:<14} | MQTT:{'ON' if mqtt_ready else 'OFF'} | ROBOT:{last_robot_status:<18} | GATE:{ready_txt:<5}   "
        )
        sys.stdout.flush()

        cv2.imshow("Monitor Kamera", frame)
        if show_mask:
            cv2.imshow("Mask (Debug)", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("\nProgram selesai.")
            break
        elif key == ord('d'):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("Mask (Debug)")

finally:
    # safety: kirim OFF saat keluar
    mqtt_publish(PAYLOAD_OFF)

    cap.release()
    cv2.destroyAllWindows()

    if client:
        try:
            client.loop_stop()
        except:
            pass
        try:
            client.disconnect()
        except:
            pass

print("🔌 Selesai.")
