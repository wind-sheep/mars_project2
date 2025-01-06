#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import time

model = YOLO(r"/home/stu/yolo/best3.pt")
bridge = CvBridge()
last_x_center = None
last_y_center = None
last_depth = None
min_depth = 10000
start_time = None
depth_frame = None
is_K_empty = True
K = np.zeros((3, 3))

def depth_callback(msg):
    global depth_frame
    try:
        depth_frame = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting depth image: {e}")

def rgb_callback_soda(msg):
    global last_x_center, last_y_center, last_depth, min_depth
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting RGB image: {e}")
        return
    if depth_frame is None:
        rospy.logwarn("Depth frame is not available yet.")
        return
    results = model(frame, conf=0.8)
    for result in results:
        for box in result.boxes:
            x_center, y_center, width, height = box.xywh[0].tolist()
            confidence = box.conf[0].item()
            class_id = int(box.cls[0].item())
            x_min = int(x_center - width / 2)
            y_min = int(y_center - height / 2)
            x_max = int(x_center + width / 2)
            y_max = int(y_center + height / 2)
            if class_id == 1:
                try:
                    depth_value = depth_frame[int(y_center), int(x_center)]
                    if not np.isfinite(depth_value):
                        continue
                    if depth_value<=min_depth:
                        min_depth=depth_value
                        last_x_center, last_y_center, last_depth = x_center, y_center, depth_value
                        rospy.loginfo("update new min depth")
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.putText(frame, f"Soda: {depth_value:.2f}m", 
                                (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"Center: ({int(x_center)}, {int(y_center)})", 
                                (int(x_center) + 10, int(y_center) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    rospy.loginfo(f"Detected Soda at x={int(x_center)}, y={int(y_center)}, Depth={depth_value:.2f}m")
                except IndexError:
                    rospy.logwarn("Depth value out of range.")
    cv2.imshow("YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User pressed 'q'")

def rgb_callback_tea(msg):
    global last_x_center, last_y_center, last_depth, min_depth
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"Error converting RGB image: {e}")
        return
    if depth_frame is None:
        rospy.logwarn("Depth frame is not available yet.")
        return
    results = model(frame, conf=0.8)
    for result in results:
        for box in result.boxes:
            x_center, y_center, width, height = box.xywh[0].tolist()
            confidence = box.conf[0].item()
            class_id = int(box.cls[0].item())
            x_min = int(x_center - width / 2)
            y_min = int(y_center - height / 2)
            x_max = int(x_center + width / 2)
            y_max = int(y_center + height / 2)
            if class_id == 2:
                try:
                    depth_value = depth_frame[int(y_center), int(x_center)]
                    if not np.isfinite(depth_value):
                        continue
                    if depth_value<=min_depth:
                        min_depth=depth_value
                        last_x_center, last_y_center, last_depth = x_center, y_center, depth_value
                        rospy.loginfo("update new min depth")
                    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    cv2.putText(frame, f"Soda: {depth_value:.2f}m", 
                                (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)
                    cv2.putText(frame, f"Center: ({int(x_center)}, {int(y_center)})", 
                                (int(x_center) + 10, int(y_center) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    rospy.loginfo(f"Detected Soda at x={int(x_center)}, y={int(y_center)}, Depth={depth_value:.2f}m")
                except IndexError:
                    rospy.logwarn("Depth value out of range.")
    cv2.imshow("YOLO Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User pressed 'q'")

def camera_info_callback(camera_info_msg):
    global K, is_K_empty
    if is_K_empty:
        K = np.array(camera_info_msg.K).reshape(3, 3)
        is_K_empty = False
        rospy.loginfo("Camera intrinsic matrix received.")

def callback(data):
    coordinates = None
    global start_time, K, last_x_center, last_y_center, last_depth
    if data.data == 1:
        rospy.loginfo("Soda")
        #global start_time, K, last_x_center, last_y_center, last_depth
        rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback_soda)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
        rospy.loginfo("YOLO Depth Detector node started. Press 'q' to exit.")
        start_time = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed_time = time.time() - start_time
            if elapsed_time > 5:
                break
            rate.sleep()
        if last_x_center is not None and last_y_center is not None and last_depth is not None:
            rospy.loginfo("")
            rospy.loginfo(f"Min distance Soda detected at: x_center={int(last_x_center)}, y_center={int(last_y_center)}, depth={last_depth}")
            rospy.loginfo("")
        else:
            rospy.loginfo("No valid Soda detected.")
        if last_x_center is not None and last_y_center is not None and last_depth is not None:
            camera_z = last_depth / 1000.0
            camera_x = camera_z * (last_x_center - K[0, 2]) / K[0, 0]
            camera_y = camera_z * (last_y_center - K[1, 2]) / K[1, 1]
            rospy.loginfo("")
            rospy.loginfo("camera coordinate:(%f,%f,%f)"%(camera_x, camera_y, camera_z))
            rospy.loginfo("")
        else:
            rospy.logwarn("No valid detection to compute 3D coordinates.")
        coordinates = [camera_x, camera_y, camera_z, last_x_center]
    elif data.data == 2:
        rospy.loginfo("Tea")
        #global start_time, K, last_x_center, last_y_center, last_depth
        rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback_tea)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
        rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)
        rospy.loginfo("YOLO Depth Detector node started. Press 'q' to exit.")
        start_time = time.time()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed_time = time.time() - start_time
            if elapsed_time > 5:
                break
            rate.sleep()
        if last_x_center is not None and last_y_center is not None and last_depth is not None:
            rospy.loginfo("")
            rospy.loginfo(f"Min distance Tea detected at: x_center={int(last_x_center)}, y_center={int(last_y_center)}, depth={last_depth}")
            rospy.loginfo("")
        else:
            rospy.loginfo("No valid Tea detected.")
        if last_x_center is not None and last_y_center is not None and last_depth is not None:
            camera_z = last_depth / 1000.0
            camera_x = camera_z * (last_x_center - K[0, 2]) / K[0, 0]
            camera_y = camera_z * (last_y_center - K[1, 2]) / K[1, 1]
            rospy.loginfo("")
            rospy.loginfo("camera coordinate:(%f,%f,%f)"%(camera_x, camera_y, camera_z))
            rospy.loginfo("")
        else:
            rospy.logwarn("No valid detection to compute 3D coordinates.")
        coordinates = [camera_x, camera_y, camera_z, last_x_center]

    else:
        rospy.loginfo("I heard %d" % data.data)        

    if coordinates:
        rospy.loginfo("Coordinates: %s" % str(coordinates))

        coordinates_msg = Float32MultiArray()
        coordinates_msg.data = coordinates
        coordinates_pub.publish(coordinates_msg)
    cv2.destroyAllWindows()

def listener():
    rospy.init_node('number_listener', anonymous=True)
    rospy.Subscriber("/number", Int32, callback)
    
    global coordinates_pub
    coordinates_pub = rospy.Publisher("/coordinates", Float32MultiArray, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()