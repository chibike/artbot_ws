import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

__home_path = "/home/chibike/Projects/ArtBot-Project/Archive/artbot_ws/src/image_processing_pkg"

def talker():
    pub = rospy.Publisher('/processed_image', Image, queue_size=10)
    rospy.init_node('publish_test_image', anonymous=True)

    bridge = CvBridge()

    image = cv2.imread(__home_path + "/images/box.png")
    print np.shape(image)
    image = cv2.resize(image, (640, 480))
    print np.shape(image)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish( bridge.cv2_to_imgmsg(image, "bgr8") )
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass