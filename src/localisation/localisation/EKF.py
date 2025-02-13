import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter as EKF
from sklearn.neighbors import NearestNeighbors #for icp

class EKF_Algorithm(Node):
    def __init__(self):
        super.__init__("EKF")
        
        #ROS topics
        

        #EKF Parameters
        self.ekf = EKF(dim_x =3, dim_z =2) # x: state, z: measurement 
        self.ekf.x = np.array([0, 0, 0]) #Initial state

        self.ekf.F = np.eye(3) # State model
        self.ekf.H = np.eye(3) # Meausrement model
         
        self.ekf.P *= 1000 # Covariance matrix
        self.ekf.Q = np.eye(3) * 0.1 # Odom noise
        self.ekf.R = np.eye(3) * 0.1 # Measurement noise

        #Robot Parameters
        self.v = 0.0
        self.omega = 0.0
        self.dt = 0.0

    def state_model(self):
        pass

    def measurement_model(self, dt, v, ):
        pass

    def