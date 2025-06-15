import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.clock import Clock
from sensor_msgs.msg import Range, Image, Imu
from geometry_msgs.msg import Vector3, TransformStamped
from geometry_msgs.msg._pose import Pose
from cv_bridge import CvBridge
import cv2
import math
import time
import numpy as np
from pymavlink import mavutil
from tf2_ros import TransformBroadcaster
from px4_msgs.msg import SensorCombined, DistanceSensor, VehicleAttitude, SensorOpticalFlow
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import csv
import pytz
from datetime import datetime

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
        # print("a")
        # Publishes
        self.of_pub = self.create_publisher(SensorOpticalFlow, '/fmu/in/sensor_optical_flow', 1024)
        self.range_pub = self.create_publisher(DistanceSensor, '/fmu/in/distance_sensor',  1024)
        # Subscribers
        self.sub_sonar = self.create_subscription(Range, '/UAV/sonar_ros/out', self.cb_sonar, 1024)
        self.sub_bottom_img = self.create_subscription(Image, '/UAV/bottom/image_raw',
                                                       self.cb_bottom_img, 10)
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub_attitude = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.cb_attitude,
            qos_profile
        )
        
        ######################### Init Record data ##########################
        # Specify the timezone for Ho Chi Minh City
        tz = pytz.timezone('Asia/Ho_Chi_Minh')
        # Get the current time in Ho Chi Minh City
        now = datetime.now(tz)
        timestamp = now.strftime("%Y-%m-%d_%H-%M-%S") 
        self.fields = ['t','dx','dy']
        self.filename = f"optical_flow_log_{timestamp}.csv"
        with open(self.filename, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fields)
            writer.writeheader()
        
        self.start_time = time.time()
        self._sonar = Range()
        self._vehicle_attitude = Imu()
        self._bottom_img = Image()
        
        self.pose_msg = Pose()
        self.optical_flow_msg = Vector3()
        
        self.old_gray = None
        self.frame = None
        self.p0 = None
        self.fps_OpFlo = 0
        self.fps_sendOpFlo = 0
        
        # Lucas-Kanade parameters
        self.lk_params = dict(winSize=(21, 21),
                        maxLevel=3,
                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        # Detect feature points parameters
        self.feature_params = dict(maxCorners=50,
                            qualityLevel=0.1,
                            minDistance=7,
                            blockSize=7)
           
        self.rate = 120.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.initialise = True
        self.initial_yaw = 0
        
        self.x_shift = 0.0 
        self.y_shift = 0.0
        self.x_list = []
        self.y_list = []
        self.quality = 0
        self.altitude = 0.1
        self.tap_coefs = [0.05, 0.2, 0.5, 0.2, 0.05] #these coefs are for exponential average filter
        self.low_pass_filter_x = lowPassFilter(self.tap_coefs)
        self.low_pass_filter_y = lowPassFilter(self.tap_coefs)
        
        timer_period = 1/self.rate  # seconds
        self.t0OpFlo = time.time()
        self.t0sendOpFlo = time.time()
        self.time_ = self.get_clock().now().nanoseconds // 1000
        self.count_time = 0
        self.count_print = 0
        self.send_opt_flo_count = 0
        self.timer = self.create_timer(timer_period, self.calculate_optical_flow)
        
    @property
    def vehicle_attitude(self):
        return self._vehicle_attitude
    
    @property
    def sonar(self):
        return self._sonar
    
    @property
    def bottom_img(self):
        return self._bottom_img
    
    def reset_pose(self, msg):
        self.yaw = 0
        self.initial_yaw = 0
        self.initialise = True
        self.position_x = 0
        self.position_y = 0
        
        
    def updateLog(self):
        list_append = [{'t':'{:.03f}'.format(self.time_/1000000),'dx':'{:.06f}'.format(self.x_shift),'dy':'{:.06f}'.format(self.y_shift)}]
        with open(self.filename, 'a') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.fields)
            writer.writerows(list_append)
            csvfile.close()
            
    def calculate_optical_flow(self):
        # self.get_logger().debug("\nSonar: {}\nIMU: {}\nCamera: {}".format(self._sonar.range, self._vehicle_attitude.q, self._bottom_img.header.stamp.sec))
        if self.frame is not None:
            t1OpFlo = time.time()
            self.fps_OpFlo = 0.99* self.fps_OpFlo + 0.01/(t1OpFlo - self.t0OpFlo)
            self.t0OpFlo = t1OpFlo   
            if self.old_gray is None:
                self.old_gray = self.frame
                self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
                self.frame = None
                return None
            try:
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, self.frame, self.p0, None, **self.lk_params)
                good_new = p1[st == 1]
                good_old = self.p0[st == 1]
                
                # Calculate quality based on flow consistency
                flow_vectors = good_new - good_old
                flow_magnitude = np.linalg.norm(flow_vectors, axis=1)
                # Quality metric: scaled based on variance
                raw_quality = np.var(flow_magnitude)
                self.quality = max(0, min(255, int(255 / (1 + raw_quality))))
                
                if len(good_new) > 0:
                    # for i, (new, old) in enumerate(zip(good_new, good_old)):
                    #     x_new, y_new = map(int, new.ravel())  # Chuyển tọa độ sang int
                    #     x_old, y_old = map(int, old.ravel())  # Chuyển tọa độ sang int
                    #     self.view_frame = cv2.circle(self.frame, (x_new, y_new), 5, (0, 255, 0), -1)
                    #     self.view_frame = cv2.line(self.frame, (x_old, y_old), (x_new, y_new), (255, 0, 0), 2)
                    # if len(self.x_list)>9:
                    #     self.x_list = [self.x_list[-1]]
                    #     self.y_list = [self.y_list[-1]]
                    # self.x_list.append(np.mean(good_old[:, 0] - good_new[:, 0]))
                    # self.y_list.append(np.mean(good_old[:, 1] - good_new[:, 1]))
                    
                    motion_x = np.mean(good_old[:, 0] - good_new[:, 0])*0.001
                    motion_y = np.mean(good_old[:, 1] - good_new[:, 1])*0.001
                    if self.altitude > 0.1:
                        if abs(motion_x) > 0:
                            # self.x_shift = self.low_pass_filter_x.filter(motion_x)
                            self.x_shift = motion_x
                        if abs(motion_y) > 0:
                            # self.y_shift = self.low_pass_filter_y.filter(motion_y)
                            self.y_shift = motion_y
                    else:
                        self.x_shift = -self.x_shift*0.1
                        self.y_shift = -self.y_shift*0.1
                else:
                    self.get_logger().warn("Crash")
                    self.x_shift = -self.x_shift*0.1
                    self.y_shift = -self.y_shift*0.1
                    self.quality = 0
            except Exception as e:
                self.get_logger().warn("False tracking!!!!!!!!!!!!: {}".format(e))
                self.x_shift = -self.x_shift*0.1
                self.y_shift = -self.y_shift*0.1
                self.quality = 0
            
            self.count_print+=1
            if self.count_print > self.fps_OpFlo/2:
                self.get_logger().info(
                        f"\n\tpickpoint: {len(good_new)},"
                        f"\n\tx_shift: {self.x_shift:.6f}, y_shift: {self.y_shift:.6f}, altitude: {self.altitude:.6f},"
                        f"\n\tQuality: {self.quality:.3f}, FPS: {int(self.fps_OpFlo)}, send FPS: {int(self.fps_sendOpFlo)}"
                    )
                self.count_print = 0
            # self.x_shift = sum(self.x_list)/len(self.x_list)
            # self.y_shift = sum(self.y_list)/len(self.y_list)
            self.updateLog()
            self.publish_optical()            
            # try:
            #     cv2.imshow("Ảnh từ ROS", self.view_frame)
            #     cv2.waitKey(1)
            # except Exception as e: 
            #     self.get_logger().warn("False display: {}".format(e))
            self.frame = None
            self.old_gray = self.frame
            self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
            self.frame = None
            return None
     
    # Preprocessing function
    def preprocess_optical_flow(self,frame):
        width_crop = 100
        height_crop = 100
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
        
    def publish_optical(self):
        t1sendOpFlo = time.time()
        self.count_time = t1sendOpFlo - self.t0sendOpFlo
        self.fps_sendOpFlo = 0.99* self.fps_sendOpFlo + 0.01/self.count_time
        self.t0sendOpFlo = t1sendOpFlo
        self.data_optical = SensorOpticalFlow()
        self.time_ = self.get_clock().now().nanoseconds // 1000
        self.data_optical.timestamp = self.time_
        self.data_optical.pixel_flow[0] = self.x_shift
        self.data_optical.pixel_flow[1] = self.y_shift
        # self.data_optical.distance_m = self.altitude
        self.data_optical.distance_m = float('nan')
        # self.data_optical.quality = random.randint(1,255)
        self.data_optical.quality = self.quality
        self.data_optical.delta_angle_available = False
        self.data_optical.distance_available = False
        # self.data_optical.delta_angle = [float(self.roll), float(self.pitch), float(self.yaw)]  # Ensure values are floats
        self.data_optical.delta_angle = [float('nan'), float('nan'), float('nan')]  # Ensure values are floats
        self.data_optical.integration_timespan_us = int(self.count_time*1e6)  # Thời gian tích phân 5 ms
        self.data_optical.min_ground_distance = 0.1
        self.data_optical.max_ground_distance = 50.0
        self.data_optical.mode = 0
        self.data_optical.max_flow_rate = 2.5
        self.data_optical.error_count = 0
        self.of_pub.publish(self.data_optical)

    def cb_attitude(self, msg: VehicleAttitude):
        """Callback for the imu sensor"""
        self.get_logger().debug("Callback for the imu sensor!")
        self._vehicle_attitude = msg
        x = msg.q[1]
        y = msg.q[2]
        z = msg.q[3]
        w = msg.q[0]
        # Roll (ϕ)
        self.roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))

        # Pitch (θ)
        self.pitch = np.arcsin(2 * (w * y - z * x))

        # Yaw (ψ)
        self.yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
    def cb_sonar(self, msg: Range):
        """Callback for the sonar sensor"""
        self.get_logger().debug("Callback for the sonar sensor!")
        self._sonar = msg
        self.altitude = self._sonar.range
        distance = DistanceSensor()
        time = int(self.get_clock().now().nanoseconds / 1000)
        distance.timestamp = time
        distance.device_id = 10092548
        #self.distance.current_distance = self.sonar_data.range
        distance.current_distance = self._sonar.range
        distance.type = 0 # khong sat dinh
        distance.orientation = 25
        distance.min_distance = self._sonar.min_range
        distance.max_distance = self._sonar.max_range
        distance.signal_quality = 50
        self.range_pub.publish(distance)
        
        
    def cb_bottom_img(self, msg: Image):
        """Callback for the rear camera"""
        self.get_logger().debug("Callback for the rear camera!")
        self._bottom_img = msg
        bridge = CvBridge()
        try:
            self.frame = bridge.imgmsg_to_cv2(self._bottom_img, "bgr8")
            self.frame = self.preprocess_optical_flow(self.frame)
        except:
            self.get_logger().warn("Can't transfer to bgr8, switch other transfer!")
        

def main(args=None):
    rclpy.init(args=args)
    optical_flow_node = OpticalFlow()
    logger = optical_flow_node.get_logger()
    logger.set_level(LoggingSeverity.INFO)
    
    rclpy.spin(optical_flow_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()