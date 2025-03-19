import cv2
import mediapipe as mp
import time
import math
import numpy as np
import threading
from pybricks.messaging import BluetoothMailboxClient, TextMailbox

# EV3 Bluetooth address
SERVER = "cc:78:ab:d7:bf:6c"

# Bluetooth initialization
client = BluetoothMailboxClient()
mbox = TextMailbox("greeting", client)

print("Establishing connection...")
client.connect(SERVER)
print("Connected!")

# Fungsi untuk mengkonversi nilai ke rentang 40 hingga 160
def convertBaseMotor(value, min_input, max_input):
    return np.clip(int(((value - min_input) / (max_input - min_input)) * 360 - 180), 40, 160)

# Fungsi untuk mengkonversi nilai ke rentang 40 hingga 160
def convertElbowMotor(value, min_input, max_input):
    return np.clip(int(((value - min_input) / (max_input - min_input)) * -50), -50, 0)

# Hand tracking class
class HandTrackingDynamic:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        self.handsMp = mp.solutions.hands
        self.hands = self.handsMp.Hands()
        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8]  # Thumb and index finger tips

    def findFingers(self, frame):
        imgRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                self.mpDraw.draw_landmarks(frame, handLms, self.handsMp.HAND_CONNECTIONS)
        return frame

    def findPosition(self, frame, handNo=0):
        self.lmsList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id, lm in enumerate(myHand.landmark):
                h, w, _ = frame.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmsList.append([id, cx, cy])
        return self.lmsList

    def findDistance(self, p1, p2):
        x1, y1 = self.lmsList[p1][1:3]
        x2, y2 = self.lmsList[p2][1:3]
        return math.hypot(x2 - x1, y2 - y1)

    def calculateHandPercentage(self):
        if len(self.lmsList) != 0:
            length = self.findDistance(4, 8)
            max_length = 300  # Max distance with open hand
            min_length = 10   # Min distance with closed hand
            return np.clip(((length - min_length) / (max_length - min_length)) * -90, -90, 0)
        return 0

# Function to handle gesture detection and data processing
def gesture_detection():
    cap = cv2.VideoCapture(0)
    detector = HandTrackingDynamic()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    global gripperOpen, HorizontalMovement, verticalMovement, horizontalMovDeg, verticalMovDeg, cx, cy

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    while True:
        ret, frame = cap.read()
        frame = detector.findFingers(frame)
        lmsList = detector.findPosition(frame)

        if len(lmsList) != 0:
            percentage = detector.calculateHandPercentage()
            gripperOpen = int(percentage)
            cx, cy = lmsList[9][1], lmsList[9][2]  # Get position of landmark 9 (middle of the hand)
            
            # Konversi cx dan cy ke derajat -180 sampai +180
            horizontalMovDeg = convertBaseMotor(cx, 100, 800)  # Adjust input range based on camera resolution
            verticalMovDeg = convertElbowMotor(cy, 50, 450)     # Adjust input range based on camera resolution

            # Display hand open percentage and position
            cv2.putText(frame, f'Hand Open: {int(percentage)}%', (10, 100), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 3)
            cv2.putText(frame, f'Pos X: {cx}, Pos Y: {cy}', (10, 150), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
            cv2.putText(frame, f'Horiz Deg: {horizontalMovDeg}', (10, 200), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)
            cv2.putText(frame, f'Vert Deg: {verticalMovDeg}', (10, 250), cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 255), 2)

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to handle Bluetooth communication
def bluetooth_communication():
    global gripperOpen, horizontalMovDeg, verticalMovDeg
    while True:
        if gripperOpen is not None:
            print(f"Sending data - Gripper: {gripperOpen}, X: {horizontalMovDeg}, Y: {verticalMovDeg}")
            mbox.send(f"{gripperOpen},{horizontalMovDeg},{verticalMovDeg}")
        time.sleep(0.1)  # Adjust sending interval

# Main function to start threads
def main():
    # Declare global variables to be shared between threads
    global gripperOpen, HorizontalMovement, verticalMovement, horizontalMovDeg, verticalMovDeg, cx, cy
    gripperOpen = 0
    HorizontalMovement = 0
    verticalMovement = 0
    horizontalMovDeg = 0
    verticalMovDeg = 0
    cx, cy = 0, 0

    # Create threads for gesture detection and Bluetooth communication
    thread1 = threading.Thread(target=gesture_detection)
    thread2 = threading.Thread(target=bluetooth_communication)

    # Start both threads
    thread1.start()
    thread2.start()

    # Join threads (this will block the main thread until both threads are done)
    thread1.join()
    thread2.join()

if __name__ == "__main__":
    main()
