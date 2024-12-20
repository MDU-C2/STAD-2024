# maked by Mohamad Hamdi Alhaji Shommo
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  # YOLOv8 for object detection
import subprocess  # For calling external warning script

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        # Subscribe to images from the 'image_topic'
        self.subscription = self.create_subscription(
            Image,
            'image_topic',  # Topic to receive images from
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # YOLOv8 Nano model for fast inference

        # CvBridge to convert ROS2 images to OpenCV format
        self.bridge = CvBridge()

    def detect_aruco_and_get_zone(self, image):
        """Detect ArUco markers and calculate the 'restricted zone'."""
        # Define dictionary for both DICT_4X4_250 and DICT_5X5_250
        aruco_dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_dict_5x5 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

        # Detector parameters
        detector_params = cv2.aruco.DetectorParameters()

        # ArUco detectors
        aruco_detector_4x4 = cv2.aruco.ArucoDetector(aruco_dict_4x4, detector_params)
        aruco_detector_5x5 = cv2.aruco.ArucoDetector(aruco_dict_5x5, detector_params)

        # Detect ArUco markers in the image from both dictionaries
        markerCorners_4x4, markerIds_4x4, _ = aruco_detector_4x4.detectMarkers(image)
        markerCorners_5x5, markerIds_5x5, _ = aruco_detector_5x5.detectMarkers(image)

        # Combine detected markers
        markerCorners = markerCorners_4x4 + markerCorners_5x5
        markerIds = np.concatenate((markerIds_4x4, markerIds_5x5)) if markerIds_4x4 is not None and markerIds_5x5 is not None else None

        if markerIds is None or len(markerIds) == 0:
            self.get_logger().info("No ArUco markers found.")
            return None

        # Draw the detected markers on the image
        cv2.aruco.drawDetectedMarkers(image, markerCorners, markerIds)

        # Check how many markers were found
        num_markers = len(markerIds)
        self.get_logger().info(f"Number of ArUco markers found: {num_markers}")

        # Calculate the centroids of each marker and define the ROI
        aruco_centroids = []
        for corners in markerCorners:
            center_x = int(np.mean([corner[0] for corner in corners[0]]))
            center_y = int(np.mean([corner[1] for corner in corners[0]]))
            aruco_centroids.append((center_x, center_y))
            # Draw a red dot at the centroid (radius 5 px)
            cv2.circle(image, (center_x, center_y), 5, (0, 0, 255), -1)

        aruco_centroids = np.array(aruco_centroids)

        # If 4 ArUco markers were found, calculate the bounding box
        if len(aruco_centroids) == 4:
            x_min = int(np.min(aruco_centroids[:, 0]))
            y_min = int(np.min(aruco_centroids[:, 1]))
            x_max = int(np.max(aruco_centroids[:, 0]))
            y_max = int(np.max(aruco_centroids[:, 1]))
            self.get_logger().info(f"Bounding Box: [(x_min, y_min) = ({x_min}, {y_min}), (x_max, y_max) = ({x_max}, {y_max})]")
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Draw the bounding box
            return (x_min, y_min, x_max, y_max)

        # If exactly 3 ArUco markers were found, calculate the missing marker
        if len(aruco_centroids) == 3:
            midpoint = np.mean(aruco_centroids, axis=0)
            self.get_logger().info(f"Midpoint of detected ArUco markers: {midpoint}")

            # Define reference markers based on quadrants
            top_left = None
            top_right = None
            bottom_left = None
            bottom_right = None

            for center in aruco_centroids:
                if center[0] < midpoint[0] and center[1] < midpoint[1]:
                    top_left = center
                elif center[0] > midpoint[0] and center[1] < midpoint[1]:
                    top_right = center
                elif center[0] < midpoint[0] and center[1] > midpoint[1]:
                    bottom_left = center
                elif center[0] > midpoint[0] and center[1] > midpoint[1]:
                    bottom_right = center

            # Calculate the missing marker's position based on the diagonal
            if top_left is None and bottom_right is not None:
                # Calculate top_left from bottom_right
                x_offset = midpoint[0] - bottom_right[0]
                y_offset = midpoint[1] - bottom_right[1]
                top_left = (midpoint[0] - x_offset, midpoint[1] + y_offset)
                aruco_centroids = np.append(aruco_centroids, [top_left], axis=0)
                self.get_logger().info(f"Top-left marker missing. Calculated position: {top_left}")

            elif top_right is None and bottom_left is not None:
                # Calculate top_right from bottom_left
                x_offset = midpoint[0] - bottom_left[0]
                y_offset = midpoint[1] - bottom_left[1]
                top_right = (midpoint[0] + x_offset, midpoint[1] + y_offset)
                aruco_centroids = np.append(aruco_centroids, [top_right], axis=0)
                self.get_logger().info(f"Top-right marker missing. Calculated position: {top_right}")

            elif bottom_left is None and top_right is not None:
                # Calculate bottom_left from top_right
                x_offset = midpoint[0] - top_right[0]
                y_offset = midpoint[1] - top_right[1]
                bottom_left = (midpoint[0] - x_offset, midpoint[1] - y_offset)
                aruco_centroids = np.append(aruco_centroids, [bottom_left], axis=0)
                self.get_logger().info(f"Bottom-left marker missing. Calculated position: {bottom_left}")

            elif bottom_right is None and top_left is not None:
                # Calculate bottom_right from top_left
                x_offset = midpoint[0] - top_left[0]
                y_offset = midpoint[1] - top_left[1]
                bottom_right = (midpoint[0] + x_offset, midpoint[1] - y_offset)
                aruco_centroids = np.append(aruco_centroids, [bottom_right], axis=0)
                self.get_logger().info(f"Bottom-right marker missing. Calculated position: {bottom_right}")

            self.get_logger().info(f"Calculated missing marker: {aruco_centroids[-1]}")

            x_min = int(np.min(aruco_centroids[:, 0]))
            y_min = int(np.min(aruco_centroids[:, 1]))
            x_max = int(np.max(aruco_centroids[:, 0]))
            y_max = int(np.max(aruco_centroids[:, 1]))
            self.get_logger().info(f"Bounding Box: [(x_min, y_min) = ({x_min}, {y_min}), (x_max, y_max) = ({x_max}, {y_max})]")
            cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            return (x_min, y_min, x_max, y_max)

        return None

    def process_results(self, results, restricted_zone):
        """Process YOLO results and check against the calculated restricted zone."""
        # Define the list of object types we are interested in
        restricted_objects = ['bicycle', 'car', 'motorcycle', 'person', 'bus', 'truck', 
                              'boat', 'cat', 'dog', 'horse', 'sheep', 'cow', 
                              'elephant', 'bear', 'zebra', 'giraffe']

        # Loop through each result and object from YOLO
        for result in results:
            for obj in result.boxes:
                # Get the object's class name (e.g., person, car, etc.)
                label = self.model.names[int(obj.cls)]

                # Only process objects that are in the restricted_objects list
                if label in restricted_objects:
                    # Get the bounding box coordinates of the object
                    x1, y1, x2, y2 = map(int, obj.xyxy[0])

                    # If there's a restricted zone, check if the object is partially or fully inside the zone
                    if restricted_zone:
                        x_min, y_min, x_max, y_max = restricted_zone

                        # Check if the object is partially or fully within the restricted zone
                        if ((x1 >= x_min and x1 <= x_max and y1 >= y_min and y1 <= y_max) or
                            (x2 >= x_min and x2 <= x_max and y2 >= y_min and y2 <= y_max)):
                            # Log and send a warning for the detected object
                            self.get_logger().info(f"Object: {label} is partially or fully IN the restricted zone.... WARNING!!!")
                            
                        else:
                            self.get_logger().info(f"Object: {label} is OUTSIDE the restricted zone")

    def image_callback(self, msg):
        # Convert ROS2 image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Log the size of the received image
        height, width, _ = frame.shape
        self.get_logger().info(f"Received image size: {width} x {height}")

        # Ensure the image is the correct size for YOLO and ArUco
        target_size = (640, 480)
        if (width, height) != target_size:
            frame = cv2.resize(frame, target_size)
            self.get_logger().info(f"Resized image to {target_size}")

        # Detect ArUco markers and get the restricted zone
        restricted_zone = self.detect_aruco_and_get_zone(frame)
        if restricted_zone:
            x_min, y_min, x_max, y_max = restricted_zone
            self.get_logger().info(f"Restricted Zone: [(x_min, y_min) = ({x_min}, {y_min}), (x_max, y_max) = ({x_max}, {y_max})]")
        else:
            self.get_logger().info("No restricted zone could be found.")

        # Run YOLOv8 analysis on the image
        results = self.model(frame)

        # Process the results and compare with the restricted zone, if present
        self.process_results(results, restricted_zone)

def main(args=None):
    rclpy.init(args=args)

    # Create the node and start the subscription
    image_subscriber = ImageSubscriber()

    # Keep the node running
    rclpy.spin(image_subscriber)

    # Shut down when done
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
