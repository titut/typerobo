import cv2

cap = cv2.VideoCapture(0)
ret, frame = cap.read()

if ret:
    print("Saving to frame.png")
    cv2.imwrite("frame.png", frame)
else:
    print("Failed to capture frame")

cap.release()
