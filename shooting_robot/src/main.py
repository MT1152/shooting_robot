import cv2
import numpy as np
import time
import serial

REAL_DIAMETER_CM   = 6.7
FOCAL_LENGTH_PX    = 621.4
SAFE_KICK_DISTANCE = 17
FRAME_WIDTH        = 640
FRAME_HEIGHT       = 480

arduino = serial.Serial('COM6', 9600, timeout=1)
time.sleep(2)

cap = cv2.VideoCapture(1)

last_send = 0
walking_sent = False
seen_frames = 0

print("System started — detecting WHITE ball...")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

        #  HSV CONVERSION 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #  WHITE COLOR MASK 
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # clean noise
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_box = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:   # ignore noise
                x, y, w, h = cv2.boundingRect(cnt)

                if area > best_area:
                    best_area = area
                    best_box = (x, y, w, h)

        now = time.time()

        # BALL FOUND 
        if best_box is not None:
            seen_frames += 1

            x, y, w, h = best_box

            cx = x + w / 2

            dist = (REAL_DIAMETER_CM * FOCAL_LENGTH_PX) / w if w > 0 else 999

            norm_x = (cx - FRAME_WIDTH / 2) / (FRAME_WIDTH / 2)
            lateral = round(norm_x * 5, 1)

            #  DRAW GREEN BOX
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cv2.putText(frame,
                        f"Dist: {dist:.1f} cm",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2)

            print(f"Ball — dist={dist:.1f} cm | lateral={lateral}")

            #  WALK START
            if seen_frames > 3 and not walking_sent:
                arduino.write(b'WALK\n')
                walking_sent = True

            #  KICK 
            if dist <= SAFE_KICK_DISTANCE:
                arduino.write(f"IK:{dist:.1f},{lateral}\n".encode())
                time.sleep(0.5)
                continue

            # UPDATE DISTANCE 
            elif now - last_send > 0.2:
                arduino.write(f"DIST:{dist:.1f}\n".encode())
                last_send = now

        else:
            seen_frames = 0

        cv2.imshow("White Ball Detection", frame)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()