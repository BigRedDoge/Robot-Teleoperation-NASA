from rclpy.node import Node
from camera.msg import Image
import cv2
import rclpy

class CameraPublisher(Node):
    """
    Publishes the camera feed to the 'camera_feed' topic
    """
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_feed', 10)
        self.timer = self.create_timer(0.1, self.publish_camera_feed)  # 10 Hz
        self.cap = cv2.VideoCapture(0)

    def publish_camera_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame_bytes = cv2.imencode('.jpg', frame)[1].tobytes()
            # Put the frame in a Image message
            msg = Image()
            msg.data = frame_bytes
            # Publish the message
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
