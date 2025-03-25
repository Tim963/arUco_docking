import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class TurtleBotAruco(Node):
    def __init__(self):
        super().__init__('turtlebot_aruco')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        self.marker_length = 0.05  # Größe des ArUco-Markers in Metern
        self.target_distance = 0.14  # Der gewünschte Abstand zum Marker
        self.focal_length = 1920  # Ein Beispielwert für die Brennweite
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)  # Instantiate the detector

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)  # Use the detector instance
            
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i] == 0:  # Nur den Marker mit ID 0 betrachten
                        # Berechne den Abstand zum Marker
                        distance = self.calculate_distance(corners[i][0])
                        center_x = np.mean(corners[i][0][:, 0])
                        center_y = np.mean(corners[i][0][:, 1])
                        
                        # Berechne den Winkel zur Bildmitte
                        angle = self.calculate_angle(center_x, cv_image.shape[1])
                        
                        # Robotersteuerung
                        self.control_turtlebot(distance, angle)

                #aruco.drawDetectedMarkers(cv_image, corners, ids)  # Draw detected markers
                #cv2.imshow("Image", cv_image)
                #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

    def calculate_distance(self, corner):
        width = np.linalg.norm(corner[0] - corner[1])
        distance = (self.marker_length * self.focal_length) / width  # Distanzberechnung
        return distance

    def calculate_angle(self, center_x, image_width):
        return (center_x - image_width / 2) / (image_width / 2)  # Normalisiere den Winkel

    def control_turtlebot(self, distance, angle):
        velocity_msg = Twist()

        # Schwellenwerte für Ausrichtung und Abstand
        alignment_threshold = 0.1  # Winkelkorrektur-Schwelle
        distance_threshold = 0.05  # Abstand-Schwelle

        # Prüfen der Position und Ausrichtung
        if abs(angle) > alignment_threshold or distance > self.target_distance + distance_threshold:
            velocity_msg.linear.x = -0.1  # Vorwärts fahren
            if abs(angle) > 0:  # Wenn wir nicht ausgerichtet sind
                velocity_msg.angular.z = -0.5 * angle  # Drehen zur Ausrichtung
                velocity_msg.linear.x = -0.1
            else:
                velocity_msg.angular.z = 0.0  # Keine Drehung notwendig
                velocity_msg.linear.x = -0.1
        else:
            # Ziel erreicht
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
          #  self.get_logger().info("Docking Complete")
        # Geschwindigkeit veröffentlichen
        self.cmd_vel_pub.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_aruco = TurtleBotAruco()
    rclpy.spin(turtlebot_aruco)
    turtlebot_aruco.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
