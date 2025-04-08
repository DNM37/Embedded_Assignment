import cv2
import numpy as np
import requests
import os
import csv
import time
import argparse
import urllib.request
import serial  # For actual serial communication
import math

# ---------------------------
# Argument Parsing (Other Arguments Only)
# ---------------------------
parser = argparse.ArgumentParser(description="ESP32-CAM YOLOv3 Person Detection & Turret Control (With Serial Support)")
parser.add_argument('--url', type=str, default='http://172.20.10.2/stream', help='ESP32-CAM video stream URL')
parser.add_argument('--confidence', type=float, default=0.5, help='Confidence threshold (0-1)')
parser.add_argument('--nms-threshold', type=float, default=0.4, help='Non-max suppression threshold')
parser.add_argument('--log-file', type=str, default='person_coordinates.csv', help='CSV file path for logging')
args = parser.parse_args()

# ---------------------------
# Hard-coded Serial Port Settings
# ---------------------------
SERIAL_PORT = "/dev/cu.usbmodem11402"  # Adjust this as needed
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to microcontroller on {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print("Could not open serial port:", e)
    ser = None

# ---------------------------
# YOLO Model Files and Download
# ---------------------------
CFG_FILE = "yolov3.cfg"
WEIGHTS_FILE = "yolov3.weights"
NAMES_FILE = "coco.names"

try:
    if not os.path.exists(CFG_FILE):
        urllib.request.urlretrieve("https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg", CFG_FILE)
    if not os.path.exists(WEIGHTS_FILE):
        urllib.request.urlretrieve("https://pjreddie.com/media/files/yolov3.weights", WEIGHTS_FILE)
    if not os.path.exists(NAMES_FILE):
        urllib.request.urlretrieve("https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names", NAMES_FILE)
except Exception as e:
    print("Error downloading YOLO files:", e)
    exit(1)

with open(NAMES_FILE, 'r') as f:
    classes = [line.strip() for line in f.readlines()]

net = cv2.dnn.readNetFromDarknet(CFG_FILE, WEIGHTS_FILE)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)

try:
    output_layer_names = net.getUnconnectedOutLayersNames()
except AttributeError:
    layer_names = net.getLayerNames()
    output_layer_indices = net.getUnconnectedOutLayers()
    output_layer_names = [layer_names[i[0]-1] if isinstance(i, np.ndarray) else layer_names[i-1] for i in output_layer_indices]

# ---------------------------
# CSV Logging
# ---------------------------
csv_file = open(args.log_file, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Relative_X", "Relative_Y"])

# ---------------------------
# Global Lock and Tracker Variables
# ---------------------------
# The code uses YOLO for the initial locking.
# After clicking on a target, a tracker is created for that bounding box.
lock_initiated = False  # Becomes True when a click requests a lock.
tracker = None          # OpenCV tracker object (CSRT or KCF).
locked_bbox = None      # Current locked bounding box from the tracker.

# Parameters:
CLICK_MATCH_THRESHOLD = 50   # Maximum pixel distance between click and detection center to initiate lock.
THRESHOLD = 0.1              # Normalized offset threshold for mapping to commands.

# ---------------------------
# Mouse Callback Function
# ---------------------------
# We record the click point if no lock is active.
click_point = None

def click_event(event, x, y, flags, param):
    global click_point, lock_initiated
    if event == cv2.EVENT_LBUTTONDOWN:
        if tracker is None:  # Only allow a new lock if no lock is active.
            click_point = (x, y)
            lock_initiated = True
            print(f"Lock request at: {click_point}")
        else:
            print("Already locked on a person. Press 'r' to reset lock.")

cv2.namedWindow("ESP32-CAM Person Detection")
cv2.setMouseCallback("ESP32-CAM Person Detection", click_event)

# ---------------------------
# Tracker Initialization Function
# ---------------------------
def create_tracker():
    try:
        return cv2.TrackerCSRT_create()
    except AttributeError:
        return cv2.TrackerKCF_create()

# ---------------------------
# Connect to the MJPEG Stream
# ---------------------------
print("Connecting to MJPEG stream...")
stream = requests.get(args.url, stream=True)
stream_iterator = stream.iter_content(chunk_size=1024)
buffer = b''

exit_flag = False

while not exit_flag:
    # Read bytes until a full JPEG frame is obtained.
    while True:
        try:
            chunk = next(stream_iterator)
        except StopIteration:
            print("Stream ended.")
            exit_flag = True
            break
        buffer += chunk
        start = buffer.find(b'\xff\xd8')  # JPEG start marker
        end = buffer.find(b'\xff\xd9')    # JPEG end marker
        if start != -1 and end != -1 and end > start:
            jpg = buffer[start:end+2]
            buffer = buffer[end+2:]
            break

    if exit_flag:
        break

    if not jpg or len(jpg) == 0:
        continue

    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        continue

    frame_height, frame_width = frame.shape[:2]

    # ---------------------------
    # If Not Locked, Run YOLO Detection for Display and Lock Request
    # ---------------------------
    if tracker is None:
        # Run YOLO detection on the frame.
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(output_layer_names)

        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = float(scores[class_id])
                if confidence >= args.confidence and classes[class_id] == "person":
                    center_x = int(detection[0] * frame_width)
                    center_y = int(detection[1] * frame_height)
                    w = int(detection[2] * frame_width)
                    h = int(detection[3] * frame_height)
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    boxes.append([x, y, w, h])
                    confidences.append(confidence)
                    class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, args.confidence, args.nms_threshold)

        # Draw all detections in green.
        for i in indices:
            i = int(i)
            x, y, w, h = boxes[i]
            cx = x + w // 2
            cy = y + h // 2
            rel_x = cx / frame_width
            rel_y = cy / frame_height
            label = f"({rel_x:.2f},{rel_y:.2f})"
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(frame, label, (x, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # If a click was requested, try to match it to one of the detections.
        if lock_initiated and click_point is not None and len(indices) > 0:
            min_dist = float('inf')
            candidate_bbox = None
            for i in indices:
                i = int(i)
                x, y, w, h = boxes[i]
                cx = x + w//2
                cy = y + h//2
                dist = math.sqrt((cx - click_point[0])**2 + (cy - click_point[1])**2)
                if dist < min_dist and dist < CLICK_MATCH_THRESHOLD:
                    min_dist = dist
                    candidate_bbox = [x, y, w, h]
            if candidate_bbox is not None:
                print(f"Locked on to person via YOLO at {candidate_bbox}")
                tracker = create_tracker()
                tracker.init(frame, tuple(candidate_bbox))
                locked_bbox = candidate_bbox
            # Clear the click request regardless.
            click_point = None
            lock_initiated = False

    else:
        # ---------------------------
        # Tracker-Based Update
        # ---------------------------
        success, bbox = tracker.update(frame)
        if success:
            locked_bbox = [int(v) for v in bbox]  # convert bbox to ints [x, y, w, h]
            x, y, w, h = locked_bbox
            # Draw the locked bounding box in red.
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,255), 2)
            cv2.putText(frame, "LOCKED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            # Compute offset from frame center.
            cx = x + w//2
            cy = y + h//2
            frame_center_x = frame_width // 2
            frame_center_y = frame_height // 2
            offset_x = (cx - frame_center_x) / (frame_width / 2)  # normalized [-1,1]
            offset_y = (cy - frame_center_y) / (frame_height / 2)  # normalized [-1,1]
            # Map offset to commands.
            if offset_x < -THRESHOLD:
                x_cmd = '0'  # move left
            elif offset_x > THRESHOLD:
                x_cmd = '1'  # move right
            else:
                x_cmd = '2'  # stop horizontal
            if offset_y < -THRESHOLD:
                y_cmd = '3'  # move up
            elif offset_y > THRESHOLD:
                y_cmd = '4'  # move down
            else:
                y_cmd = '5'  # stop vertical
            # Log the locked person's relative coordinates.
            csv_writer.writerow([cx/frame_width, cy/frame_height])
            # Send commands via serial if available; otherwise print.
            if ser is not None:
                try:
                    ser.write(x_cmd.encode())
                    ser.write(y_cmd.encode())
                    print(f"Sent commands to serial: X-axis: {x_cmd}, Y-axis: {y_cmd}")
                except Exception as e:
                    print("Error writing to serial port:", e)
            else:
                print(f"Simulated commands: X-axis: {x_cmd}, Y-axis: {y_cmd}")
        else:
            print("Tracker lost the target. Lock released.")
            tracker = None
            locked_bbox = None

    # ---------------------------
    # Display the Frame
    # ---------------------------
    cv2.imshow("ESP32-CAM Person Detection", frame)

    # ---------------------------
    # Check for Keys ('q' to quit, 'r' to reset lock)
    # ---------------------------
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        exit_flag = True
    elif key == ord('r'):
        tracker = None
        locked_bbox = None
        print("Lock reset by user.")

csv_file.close()
cv2.destroyAllWindows()
