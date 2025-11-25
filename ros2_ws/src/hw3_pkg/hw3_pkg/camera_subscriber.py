# Question 2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            #annotate aruco marker
            aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            aruco_parameters = cv2.aruco.DetectorParameters_create()

            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                cv_image, aruco_dictionary, parameters=aruco_parameters
            )

            if len(corners) > 0:
                print("Aruco marker detected")
                # Draw green bounding box around the detected marker
                for corner in corners:
                    pts = corner.reshape((-1, 2))
                    pts = np.int32(pts)
                    cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                # Draw the center of the marker as a red dot
                center = np.mean(corners[0][0], axis=0)
                center = tuple(np.int32(center))
                cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

                # Display the marker ID at the top left of the image
                for i in range(len(marker_ids)):
                    cv2.putText(
                        cv_image,
                        f"ID: {marker_ids[i][0]}",
                        tuple(np.int32(corners[i][0][0]) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (255, 55, 55),
                        2
                    )
            # Plotting each image frame
            cv2.imshow("Detected Aruco Marker", cv_image)
            cv2.waitKey(0)

            # Display the image using matplotlib
            plt.imshow(cv_image)
            plt.axis('off')  # Turn off axis
            plt.show(block=False)  # Show image without blocking
            plt.pause(0.1)  # Update the plot
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)

    # Destroy the node explicitly
    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
