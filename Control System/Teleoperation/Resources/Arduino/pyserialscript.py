import cvzone
import cv2
from cvzone.HandTrackingModule import HandDetector
from serial import Serial

cap = cv2.VideoCapture(0)
detector = HandDetector(maxHands = 1, detectionCon=0.7)
mySerial = Serial("COM5", 115200)
while True:
	success, img = cap.read()
	img = detector.findHands(img)
	lmlist, bbox = detector.findPosition(img)
	if lmlist:
		fingers = detector.fingersUp()
		#print(fingers)
		mySerial.write(fingers)
	cv2.imshow("Hyperspawn 0.0.0.0.1", img)
	cv2.waitKey(1)
    