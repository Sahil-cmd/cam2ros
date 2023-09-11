"""
Copyright (c) 2023 Sahil Panjwani

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import os

class CamPublisher(Node):
    def __init__(self):
        super().__init__('cam_node')

        # Parameters
        self.declare_parameter("fps", 20)
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("publish_uncompressed", True)
        self.declare_parameter("apply_image_processing", False)
        self.declare_parameter("roi", [0, 0, -1, -1])  # ROI as [start_row, start_col, end_row, end_col]
        self.declare_parameter("adaptive_frame_rate", True)
        self.declare_parameter("compression_quality", 90)  # From 0 (worst) to 100 (best)
        self.declare_parameter("transform.translation.x", 0.0)
        self.declare_parameter("transform.translation.y", 0.0)
        self.declare_parameter("transform.translation.z", 1.0)
        self.declare_parameter("transform.rotation.x", 0.0)
        self.declare_parameter("transform.rotation.y", 0.0)
        self.declare_parameter("transform.rotation.z", 0.0)
        self.declare_parameter("transform.rotation.w", 1.0)

        # Publishers
        if self.get_parameter("publish_compressed").value:
            self.compressed_publisher_ = self.create_publisher(CompressedImage, 'cam_image/compressed', 10)

        if self.get_parameter("publish_uncompressed").value:
            self.publisher_ = self.create_publisher(Image, 'cam_image', 10)

        self.camera_id_or_path = self.get_parameter("camera_id").value
        self.update_timer_period()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.cap = cv2.VideoCapture(self.camera_id_or_path)
        self.br = TransformBroadcaster(self)
        self.add_on_set_parameters_callback(self.on_parameters_set)

    def update_timer_period(self):
        fps = self.get_parameter("fps").value
        self.timer_period = 1.0 / fps

    def on_parameters_set(self, params):
        for param in params:
            if param.name == "fps" and self.get_parameter("adaptive_frame_rate").value:
                self.update_timer_period()
        return rclpy.node.Node.SetParametersResult(successful=True)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read from camera. Attempting to reconnect...")
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_id_or_path)
            return

        # Apply Image Processing
        if self.get_parameter("apply_image_processing").value:
            frame = cv2.Canny(frame, 100, 200)

        # Apply ROI
        start_row, start_col, end_row, end_col = self.get_parameter("roi").value
        if end_row == -1:
            end_row = frame.shape[0]
        if end_col == -1:
            end_col = frame.shape[1]
        frame = frame[start_row:end_row, start_col:end_col]
        self.get_logger().info(f"Sliced image dimensions: {frame.shape[0]}x{frame.shape[1]}")


        # Publish uncompressed Image
        if self.get_parameter("publish_uncompressed").value:
            if self.get_parameter("apply_image_processing").value:
                image_msg = self.convert_opencv_to_imgmsg(frame, "mono8")
            else:
                image_msg = self.convert_opencv_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(image_msg)

        # Compress and Publish Image
        if self.get_parameter("publish_compressed").value:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.get_parameter("compression_quality").value])
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.data = np.array(buffer).tostring()
            compressed_image_msg.format = "jpeg"
            self.compressed_publisher_.publish(compressed_image_msg)

        # Publish TF2 Transform
        self.publish_transform()

    def convert_opencv_to_imgmsg(self, cv_image, encoding="passthrough"):
        img_msg = Image()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        if encoding == "passthrough":
            if cv_image.dtype == np.uint8:
                img_msg.encoding = "bgr8"
                img_msg.step = cv_image.shape[1] * 3  # 3 bytes per pixel for bgr8
            else:
                raise NotImplementedError
        else:
            img_msg.encoding = encoding
            img_msg.step = cv_image.shape[1] * len(encoding)  # This is a rough estimation. Adjust accordingly.
        img_msg.data = cv_image.tobytes()
        return img_msg


    def publish_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "camera"
        t.transform.translation.x = self.get_parameter("transform.translation.x").value
        t.transform.translation.y = self.get_parameter("transform.translation.y").value
        t.transform.translation.z = self.get_parameter("transform.translation.z").value
        t.transform.rotation.x = self.get_parameter("transform.rotation.x").value
        t.transform.rotation.y = self.get_parameter("transform.rotation.y").value
        t.transform.rotation.z = self.get_parameter("transform.rotation.z").value
        t.transform.rotation.w = self.get_parameter("transform.rotation.w").value
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CamPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
