import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.clock import Clock
from sensor_msgs.msg import Range, Image, Imu
from geometry_msgs.msg import Vector3, TransformStamped
from geometry_msgs.msg._pose import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import math
import time
import numpy as np
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
 
def deg2rad(deg):
    rad = (3.14159/180.0) * deg
    return rad

class lowPassFilter():
	def __init__(self, tap_coefs):
		self.tap_coefs = tap_coefs
		self.len_coefs = len(tap_coefs)
		self.sample_buffer = [0]*self.len_coefs

	def filter(self, data):
		self.result = 0
		self.sample_buffer.append(data)
		if len(self.sample_buffer) > self.len_coefs:
			self.sample_buffer.pop(0)

		for i in range(self.len_coefs):
			self.result += self.tap_coefs[i]*self.sample_buffer[i]
		return self.result


class OpticalFlow(Node):
    def __init__(self):
        super().__init__('optical_flow_node')
        self.get_logger().debug("Init optical_flow_node!")
        
        # Subscribers
        self.sub_sonar = self.create_subscription(Range, '/simple_drone/sonar/out', self.cb_sonar, 1024)
        self.sub_imu = self.create_subscription(Imu, '/simple_drone/imu/out', self.cb_imu, 1024)
        self.sub_odom = self.create_subscription(Odometry, '/simple_drone/odom', self.cb_odom, 1024)
        self.sub_bottom_img = self.create_subscription(Image, '/simple_drone/bottom/image_raw',
                                                       self.cb_bottom_img, 1024)
        
        
        self._sonar = Range()
        self._imu = Imu()
        self._bottom_img = Image()
        self._odom = Odometry()
        
        self.pose_msg = Pose()
        self.optical_flow_msg = Vector3()
        
        self.old_gray = None
        self.frame = None
        self.p0 = None
        self.fps_OpFlo = 0
        
        # Lucas-Kanade parameters
        self.lk_params = dict(winSize=(21, 21),
                        maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        # Detect feature points parameters
        self.feature_params = dict(maxCorners=20,
                            qualityLevel=0.3,
                            minDistance=7,
                            blockSize=7)
        
        self.arg_filter = 9
        self.l_scale_x = [1.0 for i in range(self.arg_filter)]
        self.l_scale_y = [1.0 for i in range(self.arg_filter)]
        self.scalar_x = 1.0  #motion vector to pixel scalar
        self.scalar_y = 1.0
      
        self.rate = 60.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.odom_velocity_x = 0.0
        self.odom_velocity_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.pre_position_x = 0.0
        self.pre_position_y = 0.0
        self.altitude = 1.0 #need real time altitude from sensor just a place holder for now
        self.tap_coefs = [0.05, 0.2, 0.5, 0.2, 0.05] #these coefs are for exponential average filter
        self.low_pass_filter_x = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_y = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_roll = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_pitch = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_corrected_x = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_corrected_y = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_scale_x = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_scale_y = lowPassFilter(self.tap_coefs)
        self.initial_yaw = 0
        
        timer_period = 1/self.rate  # seconds
        self.t0OpFlo = 0
        self.count_print = 0
        self.timer = self.create_timer(timer_period, self.optical_flow_calibration)
        
    @property
    def sonar(self):
        return self._sonar

    @property
    def imu(self):
        return self._imu

    @property
    def bottom_img(self):
        return self._bottom_img
    
    @property
    def odom(self):
        return self._odom
    
    def reset_pose(self, msg):
        self.yaw = 0
        self.initial_yaw = 0
        self.initialise = True
        self.position_x = 0
        self.position_y = 0
  
    def optical_flow_calibration(self):
        self.get_logger().debug("\nSonar: {}\nIMU: {}\nCamera: {}\nOdom: \n{}".format(self._sonar.range, self._imu.angular_velocity, self._bottom_img.header.stamp.sec, self._odom))
        if self.frame is not None:
            t1OpFlo = time.time()
            self.fps_OpFlo = 0.99* self.fps_OpFlo + 0.01/(t1OpFlo - self.t0OpFlo)
            self.t0OpFlo = t1OpFlo
            
            # Update odom
            velocity_global_y =  -(self.position_x - self.pre_position_x)*self.rate
            velocity_global_x =  -(self.position_y - self.pre_position_y)*self.rate
            self.odom_velocity_x = velocity_global_y * math.sin(self.yaw) + velocity_global_x * math.cos(self.yaw)
            self.odom_velocity_y = velocity_global_y * math.cos(self.yaw) - velocity_global_x * math.sin(self.yaw)
            self.pre_position_x = self.position_x
            self.pre_position_y = self.position_y
            
            if self.old_gray is None:
                self.old_gray = self.frame
                self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
                self.frame = None
                return None
            try:
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, self.frame, self.p0, None, **self.lk_params)
                good_new = p1[st == 1]
                good_old = self.p0[st == 1]
                if len(good_new) > 0:
                    x_shift = np.mean(good_old[:, 0] - good_new[:, 0])
                    y_shift = np.mean(good_old[:, 1] - good_new[:, 1])
                                        
                    # -----------------------------------   
                    x_shift = self.low_pass_filter_x.filter(x_shift)
                    y_shift = self.low_pass_filter_y.filter(y_shift)          
                    # add rotation movement
                    delta_rotation_roll = (self.roll - self.prev_roll) * self.rate # 1/dt = rate
                    delta_rotation_pitch = (self.pitch - self.prev_pitch) * self.rate
                    self.prev_roll = self.roll
                    self.prev_pitch = self.pitch
                    delta_rotation_roll = self.low_pass_filter_roll.filter(delta_rotation_roll)
                    delta_rotation_pitch = self.low_pass_filter_pitch.filter(delta_rotation_pitch)
                    motion_x = self.odom_velocity_x - delta_rotation_pitch
                    motion_y = self.odom_velocity_y + delta_rotation_roll
                    # print("delta_rotation_pitch: {:.6f}, delta_rotation_roll: {:.6f}".format(delta_rotation_pitch,delta_rotation_roll))
                    # print("x_shift: {:.6f}, y_shift: {:.6f}".format(x_shift,y_shift))
                    sca_x = math.tan(motion_x/(self.rate * self.altitude))*500/x_shift # z * v/f, f = 500 pixels
                    sca_y = math.tan(motion_y/(self.rate * self.altitude))*500/y_shift  # scalar still needs to be determined
                    # print("sca_x: {:.6f}, sca_y: {:.6f}".format(sca_x,sca_y))
                    if (abs(motion_x)> 0.001) and (abs(sca_x)>0.001):
                        self.l_scale_x.pop(0)
                        self.l_scale_x.append(sca_x)
                        self.scalar_x = sum(self.l_scale_x)/len(self.l_scale_x)
                        self.l_scale_x[-1] = self.scalar_x
                        # self.scalar_x = 0.9*sca_x+0.1*self.scalar_x
                    else: 
                        self.l_scale_x = [1.0 for i in range(self.arg_filter)]
                        self.scalar_x = 1.0  #motion vector to pixel scalar

                    if (abs(motion_y)> 0.001) and (abs(sca_y)>0.001):
                        self.l_scale_y.pop(0)
                        self.l_scale_y.append(sca_y)
                        self.scalar_y = sum(self.l_scale_y)/len(self.l_scale_y)
                        self.l_scale_y[-1] = self.scalar_y
                        # self.scalar_y = 0.9*sca_y+0.1*self.scalar_y
                    else:
                        self.l_scale_y = [1.0 for i in range(self.arg_filter)]
                        self.scalar_y = 1.0
                        
                    # # -----------------------------------
                    # #vel to motion correct
                    # motion_corrected_x = self.low_pass_filter_corrected_x.filter(self.velocity_x)
                    # motion_corrected_y = self.low_pass_filter_corrected_y.filter(self.velocity_y)
                    
                    # # add rotation movement
                    # delta_rotation_roll = (self.roll - self.prev_roll) * self.rate # 1/dt = rate
                    # delta_rotation_pitch = (self.pitch - self.prev_pitch) * self.rate
                    # self.prev_roll = self.roll
                    # self.prev_pitch = self.pitch
                    # delta_rotation_roll = self.low_pass_filter_roll.filter(delta_rotation_roll)
                    # delta_rotation_pitch = self.low_pass_filter_pitch.filter(delta_rotation_pitch)
                    # motion_x = motion_corrected_x - delta_rotation_pitch
                    # motion_y = motion_corrected_y + delta_rotation_roll
                    
                    # sca_x = math.tan(motion_x*(1/self.rate * self.altitude))*500/x_shift # z * v/f, f = 500 pixels
                    # sca_y = math.tan(motion_y*(1/self.rate * self.altitude))*500/y_shift  # scalar still needs to be determined
                    # print("sca_x: {:.6f}, sca_y: {:.6f}".format(sca_x,sca_y))
                    # if (abs(motion_x)> 0.001) and sca_x>1.0:
                    #     self.scalar_x = self.low_pass_filter_scale_x.filter(sca_x)
                    # if (abs(motion_y)> 0.001) and sca_y>1.0:
                    #     self.scalar_y = self.low_pass_filter_scale_y.filter(sca_y)
                else:
                    self.get_logger().warn("Crash")
                    self.velocity_x = 0
                    self.velocity_y = 0
            except Exception as e:
                self.get_logger().warn("False tracking!!!!!!!!!!!!: {}".format(e))
                    
                       
            self.count_print+=1
            if self.count_print > self.fps_OpFlo/2:
                self.get_logger().info("\n\tvx: {:.3f}, vy: {:.3f}\n\tX: {:.3f}, Y: {:.3f},Z: {:.3f}\n\tscalar_x: {:.3f}, scalar_y: {:.3f}\n\tFPS: {}".format(self.odom_velocity_x, self.odom_velocity_y, 
                                                                                                                                                  self.position_x, self.position_y, self.altitude,
                                                                                                                                                  self.scalar_x, self.scalar_y,
                                                                                                                                                  int(self.fps_OpFlo)))
                self.count_print = 0
            cv2.imshow("Ảnh từ ROS", self.frame)
            cv2.waitKey(1)
            self.frame = None
            self.old_gray = self.frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
            self.frame = None
            return None
        
    # Preprocessing function
    def preprocess_optical_flow(self,frame):
        width_crop = 300
        height_crop = 300
        frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        # print(frame)
        # print(frame.shape)
        height, width = frame.shape
        # Take center image
        center_x = width // 2
        center_y = height // 2
        # Take position for crop
        start_x = center_x - (width_crop // 2)
        start_y = center_y - (height_crop // 2)
        # Take crop image
        frame = frame[start_y:start_y + height_crop, start_x:start_x + width_crop]
        return frame
    
    def publish_pose(self):
        velocity_global_x = self.velocity_x * math.cos(self.yaw) - self.velocity_y * math.sin(self.yaw)
        velocity_global_y = self.velocity_x * math.sin(self.yaw) + self.velocity_y * math.cos(self.yaw)
        self.position_x += velocity_global_x  * (1.0/self.rate)
        self.position_y += velocity_global_y  * (1.0/self.rate)
        # self.pose_pub.publish(self.pose_pub)
  
    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        self.get_logger().debug("Callback for the sonar sensor!")
        self._sonar = msg
        self.altitude = self._sonar.range

    def cb_imu(self, msg: Imu):
        """Callback for the imu sensor"""
        self.get_logger().debug("Callback for the imu sensor!")
        self._imu = msg
        self.roll = deg2rad(self._imu.angular_velocity.x)
        self.pitch = deg2rad(self._imu.angular_velocity.y)
        self.yaw = deg2rad(360.0 - self._imu.angular_velocity.z)
        # if self.initialise:
        #     self.initial_yaw = self.yaw
        #     self.initialise = False

    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        self.get_logger().debug("Callback for the rear camera!")
        self._bottom_img = msg
        bridge = CvBridge()
        self.frame = bridge.imgmsg_to_cv2(self._bottom_img, "bgr8")
        self.frame = self.preprocess_optical_flow(self.frame)
        
    def cb_odom(self, msg: Odometry):
        """Callback for the odom"""
        self.get_logger().debug("Callback for the odom!")
        self._odom = msg
        self.position_x = self._odom.pose.pose.position.x
        self.position_y = self._odom.pose.pose.position.y
        
    
        

def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = OpticalFlow()
    logger = optical_flow_node.get_logger()
    logger.set_level(LoggingSeverity.INFO)
    
    rclpy.spin(optical_flow_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()