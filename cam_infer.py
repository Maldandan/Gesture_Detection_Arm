from ultralytics import YOLO
import cv2
import math
import serial
import time
import threading

# Function to send commands to Arduino
def send_command(gesture):
    ser.write((gesture + ';').encode())
    time.sleep(0.1)  # Delay to prevent overwhelming the serial port

# Start webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Model
model = YOLO(r"C:\Users\engma\Desktop\data\yolo\best.pt")

# Object classes
classNames = ['call', 'dislike', 'fist', 'four', 'like', 'mute', 'ok', 'one', 'palm', 'peace', 'peace_inverted', 'rock', 'stop', 'stop_inverted', 'three', 'three2', 'two_up', 'two_up_inverted']

ser = serial.Serial('COM5', 9600)
if not ser.isOpen():
    ser.open()  # Open the serial connection if it's not already open

while True:
    success, img = cap.read()
    results = model(img, stream=True)

    # Coordinates
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Bounding box
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Convert to int values

            # Confidence
            confidence = math.ceil((box.conf[0] * 100)) / 100
            print("Confidence --->", confidence)

            # Class name
            cls = int(box.cls[0])
            gesture = classNames[cls]
            print("Class name -->", gesture)

            # Send gesture command to Arduino using a separate thread
            threading.Thread(target=send_command, args=(gesture,)).start()

    # Rest of your code for displaying webcam and handling key events

# Close the serial connection when done
ser.close()
