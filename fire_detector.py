import cv2
import numpy as np
import paho.mqtt.client as mqtt
import time

BROKER = "localhost"
TOPIC = "fire/detect"

client = mqtt.Client()
client.connect(BROKER, 1883, 60)

cap = cv2.VideoCapture(0)

fire_status = "OFF"

while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_fire = np.array([0, 150, 150])
    upper_fire = np.array([35, 255, 255])

    mask = cv2.inRange(hsv, lower_fire, upper_fire)
    fire_pixels = cv2.countNonZero(mask)

    if fire_pixels > 3000:
        if fire_status != "ON":
            client.publish(TOPIC, "ON")
            fire_status = "ON"
            print("🔥 FIRE ON")
    else:
        if fire_status != "OFF":
            client.publish(TOPIC, "OFF")
            fire_status = "OFF"
            print("❌ FIRE OFF")

    cv2.imshow("Fire Detection", mask)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
client.disconnect()
