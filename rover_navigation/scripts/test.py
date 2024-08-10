import cv2

rtsp_url = "rtsp://127.0.0.1:8554/test"

cap = cv2.VideoCapture(rtsp_url)
if not cap.isOpened():
    print("Failed to open video stream")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Frame", frame)
        cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
