from arm_interface.srv import Arm

import os
import time
import asyncio
import cv2 as cv
from cv_bridge import CvBridge
import yaml
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy
import argparse

import ikpy.chain
import ikpy.utils.plot as plot_utils

import numpy as np
import math

import ipywidgets as widgets
import serial

from geometry_msgs.msg import Twist

import matplotlib.pyplot as plt
from robp_interfaces.msg import DutyCycles


class MinimalService(Node):

    def __init__(self):
        super().__init__('arm_service')
        self.srv = self.create_service(Arm, 'arm', self.arm_callback)
        self.publisher = self.create_publisher(Int16MultiArray, 'multi_servo_cmd_sub', 10)
        self.img_publisher = self.create_publisher(Image, 'arm_image', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.imagesubscriber = self.create_subscription(Image, "/arm_camera/image_raw", self.image_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)) ## frame id is arm_camera_link
        self.servo_sub = self.create_subscription(JointState,'/servo_pos_publisher',self.arm_pos_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.duty_pub = self.create_publisher(DutyCycles, "/motor/duty_cycles", 10)
        self.data_sets = [[11000,12000,12000,12000,12000,12000,2000,2000,2000,2000,2000,2000],
                    [3000,12000,3000,19000,10500,12000,2000,2000,2000,2000,2000,2000],
                    [11000,12000,3000,12000,4000,12000,2000,2000,2000,2000,2000,2000],
                    [-1,12000,8000,16000,10000,12000,2000,2000,2000,2000,2000,2000],
                    [2000,12000,3000,12000,4000,14000,2000,2000,2000,2000,2000,2000]]
        self.arm_length = [0.101,0.094,0.169]
        self.latest_image = None
        self.curr_arm_pos = None

    def image_callback(self,image):
        self.latest_image = image
        self.latest_image_time = self.get_clock().now()

    def arm_pos_callback(self,arr):
        self.curr_arm_pos = arr.position
        self.curr_arm_pos_time = self.get_clock().now()

    def wait_for_fresh_image(self, after_time, timeout=2.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image_time and self.latest_image_time > after_time:
                return True
        return False

    def wait_for_fresh_joint_state(self, after_time, timeout=2.0):
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.curr_arm_pos_time and self.curr_arm_pos_time > after_time:
                return True
        return False
        
    def get_arm_pos(self):
        #jointstate = rospy.wait_for_message('/servo_pos_publisher', JointState)
        #angles = jointstate.position
        rclpy.spin_once(self,timeout_sec=1.0)
        print("Getting joint state: ")# + str(angles))
        if self.curr_arm_pos is not None:
            return self.curr_arm_pos
        else:
            return None

    def pos_ok_check(self,target_position):
        x, y, z = target_position
        print("check: " + str(target_position))
        return (
            np.all([0.14 <= y <= 0.195, -0.135 <= x <= 0.135])
            #and (-0.01 <= z <= 0.3)
        )

    def safepublish(self,arr):
        if self.pos_ok_check(self.forward_kinematics(self.transform_from_robot(arr[0:6]))):
            print("safe to grip")
            msg = Int16MultiArray()
            msg.data = arr
            self.publisher.publish(msg)
        else:
            print("bas pos")
    
    def transform_to_robot(self,ang): 
        # given all angles in radians, we compute with radians then convert and multiply with 100 to make
        # the messages the right shape to send to the robot
        rob_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        

        rob_ang[0] = round(math.degrees((ang[0] + math.radians(30)))*100) # takes the given angle and adds 30 deg, since 120 is 90

        rob_ang[1] = round(math.degrees((math.radians(210) - ang[1]))*100)

        rob_ang[2] = round(math.degrees((ang[2] + math.radians(35)))*100)

        rob_ang[3] = round(math.degrees((math.radians(210) - ang[3]))*100)  # flipped around, this should give correct angle

        rob_ang[4] = round(math.degrees((ang[4] + math.radians(35)))*100)

        rob_ang[5] = round(math.degrees((ang[5] + math.radians(30)))*100)

        return rob_ang

    def transform_from_robot(self,ang): 
        # given all angles in radians, we compute with radians then convert and multiply with 100 to make
        # the messages the right shape to send to the robot
        rob_ang = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rob_ang[0] = math.radians(ang[0]/100-30) # takes the given angle and adds 30 deg, since 120 is 90

        rob_ang[1] = math.radians(210-ang[1]/100)

        rob_ang[2] = math.radians(ang[2]/100-30)

        rob_ang[3] = math.radians(210-ang[3]/100)  # flipped around, this should give correct angle

        rob_ang[4] = math.radians(ang[4]/100-30)

        rob_ang[5] = math.radians(ang[5]/100-30)

        return rob_ang

    def inverse_kinematics(self,target_position):
        #phi_1 = math.radians(20)
        a,b,c = self.arm_length
        x, y, z = target_position # transformed to arm, change to polar and calculate first arm length
        
        if not self.pos_ok_check(target_position):
            if y < 0.12:
                return [1],[x,y] # returning too close to robot
            else:
                return [0],[x,y]
        if abs(x)>=0.01:
            theta = math.atan2(y,abs(x))
            if x<=0:
                iktheta = math.radians(180)-theta
            else:
                iktheta = theta
        
        elif abs(x)<0.01 and y>0.01:
            iktheta = math.radians(90)
        
        ikx = x # changes the x and y to account for the first link being decided 
        iky = y # we assume that theta is zero in x direction  and phi_1 zero in ground plane
        ikz = z+c # this sets z to the length of link z above object
        rho = math.sqrt(ikx**2+iky**2)
        #print("theta, rho, x, y, z is: " +str(iktheta) + ", " + str(rho)+ ", " + str(ikx)+ ", " + str(iky)+ ", " + str(ikz))

        two_joint_dist = math.sqrt(ikz**2+rho**2)
        if a+b <= two_joint_dist:
            self.get_logger().info("a + b = " + str(a+b))
            self.get_logger().info("too far: " + str(two_joint_dist))
            return [0],[ikx,iky]
        #print(((abs(ikz)**2+rho**2-b**2-a**2)/(-2*b*a)))#%math.pi) # the minus in 2bc is removed below
        phi_2 = math.acos(((abs(ikz)**2+rho**2-a**2-b**2)/(-2*a*b)))#%math.pi) 

        if ikz>=0: 
            sign = 1
        else:
            sign = -1
        phi_1 = sign*math.atan2(abs(ikz),rho) + math.asin((b*math.sin(phi_2))/ two_joint_dist)   #math.asin((math.sqrt(rho**2+ikz**2)*math.sin(phi_3+math.radians(90)))/c)-phi_1
        my_coords_phi_1 = phi_1 #+ math.radians(30)
        my_coords_phi_2 = phi_2 - math.radians(90)
        #print("phi1, phi2: "+ str(my_coords_phi_1)+ ", " + str(phi_2))

        my_coords_phi_3 = -(my_coords_phi_1+math.radians(5)+my_coords_phi_2-math.radians(90))

        ang = [math.radians(90), math.radians(90), my_coords_phi_3, my_coords_phi_2, my_coords_phi_1, iktheta] # not sure, theta may be flipped and also phi_2.
        plt_ang = [my_coords_phi_1, my_coords_phi_2, my_coords_phi_3]
        forward = self.forward_kinematics(ang)
        #print("forward kinematics gives position: "+ str(forward))
        return self.transform_to_robot(ang), plt_ang
    
    def forward_kinematics(self,angles): #all angles assumed to be 90 while standing straight out from previous link
        a,b,c = self.arm_length
        # print("angles are " + str(angles))
        rho = round(a*math.cos(angles[4])+b*math.cos(angles[4] + angles[3]-math.radians(90)),6) #+c*math.cos(angles[4]+angles[3]+angles[2]-math.radians(180)),6)
        z = round(a*math.sin(angles[4])+b*math.sin(angles[4]+angles[3]-math.radians(90)),4) #+c*math.sin(angles[4]+angles[3]+angles[2]-math.radians(180)),4)
        x = round(rho*math.cos(angles[5]),4)
        y = round(rho*math.sin(angles[5]),4)
        pos = [x,y,z]

        return pos
    
    def cam_forward_kinematics(self,angles): #all angles assumed to be 90 while standing straight out from previous link
        a,b,c = self.arm_length
        phi1 = angles[4]-math.radians(5)
        phi2 = angles[3]-math.radians(5)
        print("phi: " + str(phi1))
        # print("angles are " + str(angles))
        rho = round(a*math.cos(phi1)+b*math.cos(phi1 + phi2-math.radians(90))+0.045, 6)
        z = round(a*math.sin(phi1)+b*math.sin(phi1+phi2-math.radians(90))-0.04, 4)
        x = round(rho*math.cos(angles[5]),4)
        y = round(rho*math.sin(angles[5]),4)
        pos = [x,y]
        print(angles)
        print("cam rho, x, y is: " + str(rho) + ", " +str(x) + ", " + str(y))

        return pos
    
    def plot_robot_arm(self, link_lengths, joint_angles):
    # """
    # Plots a 2D robot arm given link lengths and user-defined joint angles.

    # :param link_lengths: List of link lengths [L1, L2, ...]
    # :param joint_angles: List of joint angles in degrees [θ1, θ2, ...] (relative to previous link)
    # """
    # Convert angles to radians
        #joint_angles = np.radians(joint_angles)

        # Initialize base pos    joint_angles = np.radians(joint_angles)

        # Initialize base position
        x_positions = [0]
        y_positions = [0]

        # Start at 90 degrees (pointing up)
        current_angle = np.pi / 2  

        # Compute forward kinematics
        for i in range(len(link_lengths)):
            current_angle += joint_angles[i]-math.radians(90)  # Add relative angle
            x_new = x_positions[-1] + link_lengths[i] * np.cos(current_angle)
            y_new = y_positions[-1] + link_lengths[i] * np.sin(current_angle)
            x_positions.append(x_new)
            y_positions.append(y_new)

        # Plot the robot arm
        plt.figure(figsize=(6,6))
        plt.plot(x_positions, y_positions, '-o', markersize=8, linewidth=3)
        plt.xlim(-sum(link_lengths), sum(link_lengths))
        plt.ylim(-sum(link_lengths), sum(link_lengths))
        plt.grid(True)
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("2D Robot Arm Visualization (User Input Angles)")
        plt.show()

    def get_color_mask(self,image):
        # Convert to HSV
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        # Define color ranges in HSV

        # Red can wrap around HSV hue range, so we usually define it in two parts
        lower_red1 = np.array([0, 100, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 70])
        upper_red2 = np.array([200, 255, 255])

        # Green range
        lower_green = np.array([35, 50, 70])
        upper_green = np.array([85, 255, 255])

        # Blue range
        lower_blue = np.array([100, 100, 0])
        upper_blue = np.array([140, 255, 255])

        # Create masks
        mask_red1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv.bitwise_or(mask_red1, mask_red2)

        mask_green = cv.inRange(hsv, lower_green, upper_green)
        mask_blue = cv.inRange(hsv, lower_blue, upper_blue)

        # Combine all masks
        mask = cv.bitwise_or(mask_red, cv.bitwise_or(mask_green, mask_blue))
        return mask

    def get_obj_pos(self,obj_class):
        pos = []
        image = self.latest_image
        
        centers = []
        all_points = []
        rect = []
        #Convert ROS Image message to OpenCV format
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        #frame = cv.imread(os.getcwd() + "/animaltop.jpg")

        with open(os.getcwd() + '/src/robp_robot/usb_cam/config/camera_info.yaml') as f:
            calib = yaml.safe_load(f)

        # Convert data to NumPy arrays
        camera_matrix = np.array(calib['camera_matrix']['data']).reshape((3, 3))
        dist_coeffs = np.array(calib['distortion_coefficients']['data'])
        undistorted = cv.undistort(frame, camera_matrix, dist_coeffs)
        blur_undistorted = cv.blur(undistorted,(3 ,3))

        #cv.imwrite('animaltop.jpg', frame)
        N = 50 # pixels to remove from bottom
        #cropped_frame = frame[:frame.shape[0] - N, :]
        cropped_frame = blur_undistorted[20:420, 60:580]   
        grey_crop = cv.cvtColor(cropped_frame, cv.COLOR_BGR2GRAY)
        #image_path = os.getcwd() + "/src/arm_service/arm_service/sphere.jpg"
        # print("Current Working Directory:", os.getcwd())
        
        
        mask = self.get_color_mask(cropped_frame)
        #contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)    
        plt.imshow(mask)
        plt.show()

        edges = cv.Canny(grey_crop,100,500)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        filtered_contours = [cnt for cnt in contours if cv.arcLength(cnt, False) > 50]
        for cnt in filtered_contours:
            M = cv.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                centers.append([cx,cy])

        cv.drawContours(cropped_frame,filtered_contours, -1, (0,255,0),3)
        for pnt in centers: 
            cv.circle(cropped_frame, (pnt), 5, (0, 0, 255), -1)
        print(centers)
        
        all_points = np.vstack(filtered_contours)
        rect = cv.minAreaRect(all_points)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        ((cx, cy), (w, h), angle) = rect

        # Force consistent edge direction
        if w < h:
            angle += 90
        
        print(rect)
        cx = round(rect[0][0])
        cy = round(rect[0][1])
        cv.drawContours(cropped_frame, [box], 0, (0, 255, 0), 2)
        cv.circle(cropped_frame, ([cx,cy]), 5, (0, 0, 255), -1)
        
        cv.imwrite('latest2.jpg', cropped_frame)
        # if len(centers)>=2:
        #     self.frame_PCA(cropped_frame,centers)
        # elif len(centers) == 1:
        #     shape = self.detect_shape(filtered_contours) #max(filtered_contours, key=cv.contourArea))
        #     print(shape)

        tmpx = round((cx/520-0.5)*20-1,3)/100#round((cntr[0]/640-0.5)*30,3)/100
        tmpy = round(((1-cy/400)-0.5)*10+1,3)/100#round(((1-cntr[1]/430)-0.5)*20+0.01,3)/100
        #cntr_pos.append([tmpx,tmpy])
        print("x,y: "+str(tmpx) + ", " + str(tmpy))
        det_img = Image()
        det_img = bridge.cv2_to_imgmsg(cropped_frame, encoding='bgr8')
        self.img_publisher.publish(det_img)
        
        if math.sqrt(tmpx**2+tmpy**2)<= 0.15:
            
            plt.subplot(2,2,1)
            plt.imshow(mask)
            plt.subplot(2,2,2)
            plt.imshow(edges,cmap='gray')
            plt.subplot(2,2,3)
            plt.imshow(cropped_frame)
            plt.show()
            return [tmpx,tmpy],angle
        

        return []

    def frame_PCA(self, contour, img):
        # Flatten and convert to np.array
        data_pts = np.array(contour, dtype=np.float32).reshape(-1, 2)
        mean = np.mean(data_pts, axis=0)
        print("Contour shape:", np.array(contour).shape)
        # Subtract mean (center the points)
        centered = data_pts - mean

        # PCA
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov)

        # First principal component
        pc1 = eigenvectors[:, 0]
        if pc1[0] < 0:
            pc1 = -pc1

        angle_rad = np.arctan2(pc1[1], pc1[0])
        angle_deg = np.degrees(angle_rad) % 180
        print("angles deg, rad: " + str(angle_deg) + ", " + str(angle_rad))

        plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
        
        plt.quiver(mean[0], mean[1], pc1[0], pc1[1], angles='xy', scale_units='xy',
                   scale=0.01, color='r', width=0.015, label='PC1')
        plt.title(f"PCA Angle: {round(angle_deg, 2)}°")
        plt.axis('equal')
        plt.legend()
        plt.gca().invert_yaxis() 
        plt.show()
        return angle_deg

    def detect_shape(self,contours):
        # Find contours
        for cnt in contours:
            print("eplsilon:" + str(0.01 * cv.arcLength(cnt, True),))
            approx = cv.approxPolyDP(cnt, 0.01 * cv.arcLength(cnt, True), True)
            area = cv.contourArea(cnt)
            print("area, approx: " + str(area) + ", " + str(approx))

            if area > 5:  # filter out noise
                if len(approx) > 8:
                    print("Probably a sphere (round shape)")
                    return("sphere")
                elif 4 <= len(approx) <=8:
                    print("Probably a cube (polygonal shape)")
                    return("cube")

    def get_center_from_bounding_rect(self, contour):
        x, y, w, h = cv.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        return (cx, cy, x, y, w, h)

    def cornerHarris_demo(self,val,src_gray):
            thresh = val    # Detector parameters
            blockSize = 2
            apertureSize = 3
            k = 0.05    # Detecting corners
            dst = cv.cornerHarris(src_gray, blockSize, apertureSize, k)    # Normalizing
            dst_norm = np.empty(dst.shape, dtype=np.float32)
            cv.normalize(dst, dst_norm, alpha=0, beta=255, norm_type=cv.NORM_MINMAX)
            dst_norm_scaled = cv.convertScaleAbs(dst_norm)    # Drawing a circle around corners

            for i in range(dst_norm.shape[0]):
                for j in range(dst_norm.shape[1]):
                    if int(dst_norm[i,j]) > thresh:
                        cv.circle(dst_norm_scaled, (j,i), 5, (0), 2)    # Showing the result
            #cv.namedWindow(corners_window)
            #cv.imshow(corners_window, dst_norm_scaled)# Load source image and convert it to gray
            return dst_norm_scaled

    def merge_centers(self,centers, distance_threshold=30):
        merged = []
        used = [False] * len(centers)

        for i in range(len(centers)):
            if used[i]:
                continue
            group = [centers[i]]
            used[i] = True
            for j in range(i + 1, len(centers)):
                if not used[j]:
                    dist = np.linalg.norm(np.array(centers[i]) - np.array(centers[j]))
                    if dist < distance_threshold:
                        group.append(centers[j])
                        used[j] = True
            # Average the group
            avg = tuple(np.mean(group, axis=0).astype(int))
            merged.append(avg)
        return merged

    def arm_move_check(self,actual_arr,wanted_arr,response):
        arm_diff = sum(actual_arr-wanted_arr)
        if arm_diff <= 200:
            response.success = True
            response.message = "Arm in position"
            
            return response 
        else:
            response.success = False
            response.message = "Arm not moving correctly"
            return response

    def get_obj_grip(self,obj_class):
        if obj_class == "cube" or obj_class == "sphere":
            return 11000
        elif obj_class == "animal":
            return 12000

    def angle_cube_to_grip(self,angle):
        grip_angle = round((angle+30)*100)
        return grip_angle
    
    def angle_animal_to_grip(self, angle):
        grip_angle = round(math.degrees((math.radians(210)-angle))*100)
        return grip_angle

    def drive_to_obj(self,pos):
        a,b,c =self.arm_length
        x,y = pos
         # Robot paramters
        wheel_radius = 0.046 # 0.04915
        base = 0.3 # 0.30
        dist = math.sqrt(x**2+y**2) # distance to point
        
        # Maximum velocities
        max_factor = 1 / 3
        max_vel = wheel_radius * max_factor # m/s
        max_rot = ((wheel_radius / base) / (np.pi/2)) * max_factor # rad/s
        
        

        # if object far forward
        # elIf object far left/right --> rotate l/r
        #   then drive forward if too fowrard
        # call self again and again until object can be reached
        if abs(x) <= 0.07 and dist > a+b: # drive forward
            # send command to drive straight 1 cm?
            linear_velocity = max_vel*0.4
            angular_velocity = 0.0
        elif abs(x) <= 0.07 and y  <= 0.14 :
            #send command to back up 1 cm?
            linear_velocity = -max_vel*0.4
            angular_velocity = 0.0
        else:
            if x>= 0.07:
                #rotate right 5 deg
                #then restart, drive_to_obj(get_obj_pos)
                angular_velocity = -max_rot
                linear_velocity = 0.0
            elif x<= 0.07:
                #rotate left 5 deg
                #then restart, drive_to_obj(get_obj_pos)
                angular_velocity = max_rot
                linear_velocity = 0.0

        twist_msg = Twist()
        twist_msg.linear.z = max_factor
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)
        print("driving lin, ang: " + str(linear_velocity) + ", " + str(angular_velocity))
        time.sleep(0.2) # may need another one if nothing happens
        twist_msg = Twist()
        self.cmd_vel_pub.publish(twist_msg)
        print("Stopping")

        return


        #either this, or we can look at the pos and set values accordingly.

        

    def arm_callback(self, request, response):
        time_data_set = [2000,2000,2000,2000,2000,2000]
        
        msg = Int16MultiArray()
        obj_class = request.obj_class
        grip_size = self.get_obj_grip(obj_class)

        if request.command == 1: # this is command from client
            self.get_logger().info('moving arm to top')
            msg.data = self.data_sets[int(request.command-1)]
            self.publisher.publish(msg)
        elif request.command == 2:
            self.get_logger().info('moving arm to look')
            print(int(request.command-1))
            msg.data = self.data_sets[int(request.command-1)]
            self.publisher.publish(msg)

            response.success = True 
            response.message = 'successful'
            return response
        elif request.command == 4: 
            self.get_logger().info('moving arm to drop')
            msg.data = self.data_sets[int(request.command-1)]
            self.publisher.publish(msg)
            time.sleep(2.0)
            msg.data[0] = 2000
            self.publisher.publish(msg)


            # either keep as is and make sure planner is good enough
            # otherwise, move arm to look for box
            # move if needed
            # Move object to drop
            # open gripper
        elif request.command == 6:
            #arm_pos = self.get_arm_pos()
            
            #response = self.arm_move_check(arm_pos,self.data_sets[1],response)
            # if not response.success:
            #     return response
            # else:
            #     self.get_logger().info(response.message)
            #cv.imwrite("sphere.jpg")
            #rclpy.spin_once(self,timeout_sec=1.0)
            #move_time1 = self.get_clock().now()
            #self.wait_for_fresh_image(move_time1)

            cam_obj_pos, obj_angle = self.get_obj_pos(obj_class)
            grip_angle_cube = self.angle_cube_to_grip(obj_angle)
            grip_angle_animal = self.angle_animal_to_grip(obj_angle)
            print("cam obj pos :" + str(cam_obj_pos) + ", " + str(grip_angle_cube))
            if cam_obj_pos == []:
                response.success = False 
                response.message = "No object detected"
                return response 
            
            obj_pos = self.cam_forward_kinematics(self.transform_from_robot(self.data_sets[1]))
            obj_pos[0] += float(cam_obj_pos[0])
            obj_pos[1] += float(cam_obj_pos[1]) ## this will return a position x,y which the camera sees, should be transformed to arm_base
            obj_pos.append(-0.17)
            print("obj pos: " + str(obj_pos))
            cam_angles, garbage = self.inverse_kinematics(obj_pos)

            if cam_angles == [0]:
                response.success = False
                response.message = "Object too far"
                response.xyfix = garbage
                return response
            elif cam_angles == [1]:
                response.success = False
                response.message = "Object too close"
                response.xyfix = garbage
                return response
            
            print(cam_angles)
            
            cam_angles[0] = 2000
            if obj_class == "sphere":
                cam_angles[1] = 12000
            elif obj_class == "cube" or obj_class == "animal":
                cam_angles[1] = grip_angle_cube
            print("cam_angles: "+str(cam_angles))
            cam_data_set = np.concatenate((cam_angles,time_data_set))
            
            self.get_logger().info("computed cam sequence is " + str(cam_data_set))
            
            #await asyncio.sleep(2)
            move_time = self.get_clock().now()

            self.safepublish(cam_data_set)
            time.sleep(3.0)
            response.success = True
            response.message = "Object grabbed"

        elif request.command == 7:
            #self.wait_for_fresh_joint_state(move_time)
            arm_ok = self.arm_move_check(self.curr_arm_pos[0:6],request.arm_pos,response)
            if arm_ok:
                grab_data_set = np.concatenate((request.arm_pos,time_data_set))
            
            print(grab_data_set)
            grab_data_set[0]=grip_size
            grab_data_set[6]=1000
            msg.data = grab_data_set.astype(np.int16).tolist()
            print("Grabbing")
            self.publisher.publish(msg)
            time.sleep(2.0)
            #await asyncio.sleep(2)
            msg.data = self.data_sets[0]
            print("Raising arm")
            self.publisher.publish(msg)
        elif request.command == 8:
            self.drive_to_obj(request.xy)
            response.success = True
            response.message = "Drove once"
        else:
            response.success = False
            response.message = "Send valid input please. (2, 4 or 6)"
        
        
        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.command))
        response.success = True#"success"
        response.message = 'successful'
        if request.command == 6:
            response.arm_pos = cam_data_set[0:6]
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()
    
    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

    # to do:
    # decide if we use planner or move by ourselves
    # create the math needed to move or send message
    # test the drop function MemoryError
    # implement
