import cv2
import numpy as np
import rospy
from picamera2 import Picamera2, Preview
from std_msgs.msg import Float32MultiArray


def detect_red_object(frame):
    # ===================================
    # Color Detection
    # ===================================
    hsv = cv2.cvtColor(cv2.COLOR_BGR2HSV)  # HACK: why?
    # Define range of red color in HSV
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # Find contours HACK: how?
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # ===================================
    # Find and Mark the Object
    # ===================================
    for contour in contours:
        # Calculate the center of the contour
        area = cv2.contourArea(contour)
        if area > 500:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            return (x + w / 2, y + h / 2)  # Return the center of object

    return None


def main():
    # ===================================
    # Initialize ROS Node and Publisher
    # ===================================
    rospy.init_node("camera_node", anonymous=True)
    pub = rospy.Publisher("object_position", Float32MultiArray, queue_size=10)

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration()
    picam2.configure(camera_config)
    picam2.start()

    while not rospy.is_shutdown():
        # ===================================
        # Capture and Process Frame
        # ===================================
        frame = picam2.capture_array()  # Capture a frame as a numpy array
        cv_image = cv2.cvtColor(
            frame, cv2.COLOR_RGB2BGR
        )  # Convert RGB to BGR format used by OpenCV

        position = detect_red_object(cv_image)
        if position:
            msg = Float32MultiArray(data=[position[0], position[1]])
            pub.publish(msg)

        cv2.imshow("Camera", cv_image)
        if cv2.waitKey(1) == ord("q"):  # Exit loop when 'q' is pressed
            break

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
