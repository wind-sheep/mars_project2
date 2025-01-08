# import cv2
# from ultralytics import YOLO

# # Load your YOLO model
# model = YOLO(r"C:\Users\stu10\yolov8\runs\detect\train\weights\best.pt")  # Replace with your trained model file

# # Open the camera
# camera = cv2.VideoCapture(0)  # 0 is the default camera. Change if you use an external camera.

# if not camera.isOpened():
#     print("Error: Unable to access the camera.")
#     exit()

# try:
#     while True:
#         # Capture a frame from the camera
#         ret, frame = camera.read()
#         if not ret:
#             print("Error: Unable to capture frame.")
#             break

#         # Perform object detection on the frame
#         results = model(frame,conf=0.8)

#         # Loop through detected objects
#         for result in results:
#             boxes = result.boxes  # Bounding boxes
#             for box in boxes:
#                 # Extract bounding box details
#                 x_center, y_center, width, height = box.xywh[0].tolist()  # Get bounding box center and size
#                 confidence = box.conf[0].item()  # Confidence score
#                 class_id = int(box.cls[0].item())  # Class ID
                
#                 # Calculate corners of the bounding box
#                 x_min = int(x_center - width / 2)
#                 y_min = int(y_center - height / 2)
#                 x_max = int(x_center + width / 2)
#                 y_max = int(y_center + height / 2)
                
#                 # Draw the bounding box on the frame
#                 cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
#                 cv2.putText(frame, f"Class {class_id} ({confidence:.2f})", 
#                             (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
#                 print(f"Detected object: Class={class_id}, Confidence={confidence:.2f}")
#                 print(f"Bounding Box: x_min={x_min}, y_min={y_min}, x_max={x_max}, y_max={y_max}")
#                 print(f"Bounding Box: x_center={int(x_center)}, y_center={int(y_center)}, width={int(width)}, height={int(height)}")

#         # Display the frame
#         cv2.imshow("YOLO Detection", frame)

#         # Break the loop on 'q' key press
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# finally:
#     # Release the camera and close windows
#     camera.release()
#     cv2.destroyAllWindows()

import cv2
import time
from ultralytics import YOLO

# Load your YOLO model
model = YOLO(r"C:\Users\stu10\yolov8\runs\detect\train\weights\best.pt")  # Replace with your trained model file

# Open the camera
camera = cv2.VideoCapture(0)  # 0 is the default camera. Change if you use an external camera.

if not camera.isOpened():
    print("Error: Unable to access the camera.")
    exit()

# Record start time
start_time = time.time()

try:
    last_x_center = None
    last_y_center = None
    
    while True:
        # Capture a frame from the camera
        ret, frame = camera.read()
        if not ret:
            print("Error: Unable to capture frame.")
            break

        # Perform object detection on the frame
        results = model(frame, conf=0.8) # confidence can be changed here

        # Loop through detected objects
        for result in results:
            boxes = result.boxes  # Bounding boxes
            for box in boxes:
                # Extract bounding box details
                x_center, y_center, width, height = box.xywh[0].tolist()  # Get bounding box center and size
                confidence = box.conf[0].item()  # Confidence score
                class_id = int(box.cls[0].item())  # Class ID
                
                # Save the last detected x_center and y_center
                last_x_center = int(x_center)
                last_y_center = int(y_center)
                
                # Calculate corners of the bounding box
                x_min = int(x_center - width / 2)
                y_min = int(y_center - height / 2)
                x_max = int(x_center + width / 2)
                y_max = int(y_center + height / 2)
                
                # Draw the bounding box on the frame
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(frame, f"Class {class_id} ({confidence:.2f})", 
                            (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                print(f"Detected object: Class={class_id}, Confidence={confidence:.2f}")
                print(f"Bounding Box: x_min={x_min}, y_min={y_min}, x_max={x_max}, y_max={y_max}")
                print(f"Bounding Box: x_center={int(x_center)}, y_center={int(y_center)}, width={int(width)}, height={int(height)}")

        # Display the frame
        cv2.imshow("YOLO Detection", frame)

        # Stop after 5 seconds
        elapsed_time = time.time() - start_time
        if elapsed_time > 5:
            break
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After 5 seconds, print the last recorded x_center and y_center
    if last_x_center is not None and last_y_center is not None:
        print(f"Last detected x_center: {last_x_center}, y_center: {last_y_center}")

finally:
    # Release the camera and close windows
    camera.release()
    cv2.destroyAllWindows()
