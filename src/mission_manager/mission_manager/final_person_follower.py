#!/usr/bin/env python3
"""
Person Follower ROS2 Node
Uses OpenCV DNN (MobileNet SSD) — no MediaPipe, no version conflicts.

Download models once:
  wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.caffemodel
  wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.prototxt

Place both files next to this script.
"""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# ── Model paths ───────────────────────────────────────────────────────────────
PROTOTXT   = '/home/vyshnav-si3514/Documents/aws_bot/src/mission_manager/mission_manager/MobileNetSSD_deploy.prototxt'
CAFFEMODEL = '/home/vyshnav-si3514/Documents/aws_bot/src/mission_manager/mission_manager/MobileNetSSD_deploy.caffemodel'

# MobileNet SSD class labels — index 15 is 'person'
CLASSES = [
    'background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle',
    'bus', 'car', 'cat', 'chair', 'cow', 'diningtable', 'dog',
    'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa',
    'train', 'tvmonitor'
]
PERSON_CLASS_ID = 15
CONFIDENCE_THRESHOLD = 0.5


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        self.bridge = CvBridge()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/kinect_camera/image_raw', self.callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/kinect_camera/depth/image_raw', self.depth_callback, 10)

        # Publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self.depth_image     = None
        self.person_detected = False
        self.depth_mm        = 0
        self.last_error      = 0.0
        self.last_depth      = 0.0
        self.cv_image        = None

        # Load OpenCV DNN model
        self.get_logger().info('Loading MobileNet SSD model...')
        self.net = cv2.dnn.readNetFromCaffe(PROTOTXT, CAFFEMODEL)
        self.get_logger().info('Model loaded. PersonFollower ready.')

    # ── Depth callback ────────────────────────────────────────────────────────
    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    # ── RGB callback ──────────────────────────────────────────────────────────
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        h, w = self.cv_image.shape[:2]
        image_center = w / 2.0

        # ── Run detection ─────────────────────────────────────────────────────
        blob = cv2.dnn.blobFromImage(
            cv2.resize(self.cv_image, (300, 300)),
            0.007843, (300, 300), 127.5
        )
        self.net.setInput(blob)
        detections = self.net.forward()

        # Find the highest-confidence person detection
        best_conf   = 0.0
        best_box    = None

        for i in range(detections.shape[2]):
            class_id   = int(detections[0, 0, i, 1])
            confidence = float(detections[0, 0, i, 2])

            if class_id != PERSON_CLASS_ID:
                continue
            if confidence < CONFIDENCE_THRESHOLD:
                continue
            if confidence < best_conf:
                continue

            best_conf = confidence
            x1 = int(detections[0, 0, i, 3] * w)
            y1 = int(detections[0, 0, i, 4] * h)
            x2 = int(detections[0, 0, i, 5] * w)
            y2 = int(detections[0, 0, i, 6] * h)
            best_box = (x1, y1, x2, y2)

        # ── Person found ──────────────────────────────────────────────────────
        if best_box is not None:
            self.person_detected = True
            x1, y1, x2, y2 = best_box

            # Centroid of bounding box
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2

            # Draw bounding box and centroid
            cv2.rectangle(self.cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(self.cv_image, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(self.cv_image, (cx - 15, cy), (cx + 15, cy), (255, 0, 0), 3)
            cv2.line(self.cv_image, (cx, cy - 15), (cx, cy + 15), (255, 0, 0), 3)
            cv2.line(self.cv_image, (w // 2, 0), (w // 2, h), (0, 0, 255), 1)
            label = f'Person: {best_conf:.2f}'
            cv2.putText(self.cv_image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Depth at centroid
            if self.depth_image is not None:
                cy_clamped = min(cy, self.depth_image.shape[0] - 1)
                cx_clamped = min(cx, self.depth_image.shape[1] - 1)
                self.depth_mm = float(self.depth_image[cy_clamped, cx_clamped])
            else:
                self.vel_control(0.0, 0.0)

            self.move_robot(cx, image_center)

        # ── No person ─────────────────────────────────────────────────────────
        else:
            if self.person_detected:
                # Spin to search
                self.vel_control(0.0, -0.5)
                cv2.putText(self.cv_image, 'Searching...', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                self.vel_control(0.0, 0.0)

        cv2.imshow('Person Follower', self.cv_image)
        cv2.waitKey(3)

    # ── PD controller ─────────────────────────────────────────────────────────
    def move_robot(self, cx, image_center):
        Kp_l   = 0.4
        Kp_yaw = 0.00065
        Kd_yaw = 0.00007
        Kd_l   = 0.37

        x_error = cx - image_center - 3.0

        if self.depth_mm > 3:
            P_x   = Kp_l * self.depth_mm
            P_yaw = -(Kp_yaw * x_error)
            D_yaw = ((x_error       - self.last_error) / 0.6) * Kd_yaw
            D_l   = ((self.depth_mm - self.last_depth) / 0.6) * Kd_l

            self.last_error = x_error
            self.last_depth = self.depth_mm
            self.vel_control(P_x + D_l, P_yaw + D_yaw)

            if x_error > 10:
                direction = 'Right ==>'
            elif x_error < -10:
                direction = '<== Left'
            else:
                direction = 'Centre'

            cv2.putText(self.cv_image, direction, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 0), 2)
            cv2.putText(self.cv_image, f'Depth: {self.depth_mm:.2f}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        else:
            self.vel_control(0.0, 0.0)
            cv2.putText(self.cv_image, 'Too close / no depth', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def vel_control(self, vel_x, vel_spin):
        msg = Twist()
        msg.linear.x  = float(vel_x)
        msg.angular.z = float(vel_spin)
        self.velocity_publisher.publish(msg)


def main():
    rclpy.init()
    node = PersonFollower()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()