import rospy
from camera.msg import Image
import cv2


"""
Publishes the camera feed to the 'camera_feed' topic
"""
def publish_camera_feed():
    # Initialize the publisher
    pub = rospy.Publisher('camera_feed', Image, queue_size=10)

    # Initialize the node
    rospy.init_node('camera_publisher', anonymous=True)

    # Open the camera
    cap = cv2.VideoCapture(0)

    # Set the rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Capture a frame
        ret, frame = cap.read()

        if ret:
            # Convert the frame to a ROS Image message
            image = cv2.imencode('.jpg', frame)[1]
            image_bytes = image.tobytes()

            msg = Image()
            msg.data = image_bytes

            # Publish the message
            pub.publish(msg)

        # Sleep for the remaining time
        rate.sleep()

    # Release the camera
    cap.release()


if __name__ == '__main__':
    try:
        publish_camera_feed()
    except rospy.ROSInterruptException:
        pass