#!/usr/bin/env python2.7
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import time

def read_coordinates_from_file(file_path):
    try:
        with open(file_path, 'r') as file:
            line = file.readline().strip()
            print("Raw file content:", repr(line))

            if line:
                if "Received coordinates:" in line:
                    line = line.split("Received coordinates:")[-1].strip()

                line = line.strip('()').strip()
                parts = [part.strip() for part in line.split(',')]
                if len(parts) == 4:
                    try:
                        camera_x = float(parts[0])
                        camera_y = float(parts[1])
                        camera_z = float(parts[2])
                        pixel_x = float(parts[3])
                        return (camera_x, camera_y, camera_z, pixel_x)
                    except ValueError as e:
                        rospy.logerr("Error converting coordinates to float: %s" % e)
                        return None
                else:
                    rospy.logwarn("Incorrect number of coordinates in file.")
                    return None
            else:
                rospy.logwarn("No valid coordinates in file.")
                return None
    except Exception as e:
        rospy.logerr("Failed to read coordinates from file: %s" % e)
        return None

def move_to_pose(x, y, z, ox, oy, oz, ow):
    pose_target = Pose()
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = ow
    group.set_pose_target(pose_target)
    success = group.go(wait=True)
    if success:
        rospy.loginfo("Successfully moved to Cartesian pose: ({}, {}, {})".format(x, y, z))
    else:
        rospy.logwarn("Failed to move to Cartesian pose: ({}, {}, {})".format(x, y, z))

def move_to_joint_angles(joint_angles):
    group.set_joint_value_target(joint_angles)
    success = group.go(wait=True)

    if success:
        rospy.loginfo("Successfully moved to joint angles: {}".format(joint_angles))
    else:
        rospy.logwarn("Failed to move to joint angles: {}".format(joint_angles))

def control_gripper(action):
    pub.publish(action)
    if action:
        rospy.loginfo("Gripper closed.")
    else:
        rospy.loginfo("Gripper opened.")
    time.sleep(1)

if __name__ == '__main__':
    rospy.init_node('mixed_control', anonymous=False)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("tm_arm")
    pub = rospy.Publisher('gripper/cmd_gripper', Bool, queue_size=10)
    file_path = "write.txt"
    coordinates = read_coordinates_from_file(file_path)
    if coordinates:
        camera_x, camera_y, camera_z, pixel_x = coordinates
        rospy.loginfo("Camera coordinates: x={}, y={}, z={}".format(camera_x, camera_y, camera_z))
        rospy.loginfo("Pixel x: {}".format(pixel_x))
    else:
        rospy.logwarn("Failed to read coordinates.")
        
    #change camera coordinate to arm coordinate
    arm_x, arm_y, arm_z = camera_z, -camera_x, -camera_y 
    arm_y+=0.060/238*(pixel_x-320) # a modify term due to the error when the object is far from the middle of the camera, 320 is the middle x pixel of the camera
    modified_x = -0.113                     #translation matrix of x
    modified_y = -0.018 - 0.015 -0.015      #translation matrix of y
    modified_z = 0.094 + 0.04               #translation matrix of z
    cartesian_targets = [
        [0.60 + arm_x + modified_x - 0.1, -0.12 + arm_y + modified_y, 0.69 + arm_z + modified_z, 0.5, 0.5, 0.5, 0.5], # 1st point
        [0.60 + arm_x + modified_x      , -0.12 + arm_y + modified_y, 0.69 + arm_z + modified_z, 0.5, 0.5, 0.5, 0.5]  # 2nd point
    ]

    joint_targets = [
        [0.011565, -0.248447, 2.321848, -2.072889, 1.559083, 0.000320], # above observe point       # 3rd point
        #[0.011341, 0.060872, 2.494978, -2.556190, 1.559743, 0.000165], # observe point             
        [-0.342614, 0.499793, 1.296339, -1.794004, 1.913511, 0.002463], # above place point         # 4th point
        [-0.342602, 0.545972, 1.393131, -1.937829, 1.914287, 0.001997], # place point               # 5th point
        [-0.494512, 0.216017, 1.865652, -2.078784, 2.066935, 0.001212], # go out from place point   # 6th point
        [0.011565, -0.248447, 2.321848, -2.072889, 1.559083, 0.000320], # above observe point       # 7th point
        [0.011341, 0.060872, 2.494978, -2.556190, 1.559743, 0.000165]   # observe point             # 8th point
    ]
    try:
        # Cartesian movements
        for i, pose in enumerate(cartesian_targets):
            rospy.loginfo("Moving to Cartesian pose {}".format(i + 1))
            move_to_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6])
            time.sleep(2)  # Pause between movements

            # Perform grasp actions at specific points
            if i == 1:  # 2nd point
                rospy.loginfo("Closing gripper to grasp the object.")
                control_gripper(True)  # Close the gripper
                time.sleep(2)

        # Joint angle movements
        for i, joint_angles in enumerate(joint_targets):
            rospy.loginfo("Moving to joint angles {}".format(i + 3))
            move_to_joint_angles(joint_angles)
            time.sleep(2)  # Pause between movements

            # Perform grasp actions at specific joint configurations
            if i == 2:  # 5th point
                rospy.loginfo("Opening gripper to release the object.")
                control_gripper(False)  # Open the gripper
                time.sleep(2)
    except rospy.ROSInterruptException:
        rospy.logwarn("ROS Interrupt Exception. Shutting down.")
    except KeyboardInterrupt:
        rospy.logwarn("Keyboard Interrupt. Exiting.")
    finally:
        moveit_commander.roscpp_shutdown()
