import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge, CvBridgeError

__home_path = "/home/chibike/Projects/ArtBot-Project/Archive/artbot_ws/src/image_processing_pkg"

def convert_to_js_image_data(image):
    image_shape = image.shape
    image = image.ravel()
    image_data = []
    
    for i in xrange(image_shape[0]*image_shape[1]):
        image_data.append(image[i])
        image_data.append(image[i])
        image_data.append(image[i])
        image_data.append(255)

    return image_data

def talker():
    #pub = rospy.Publisher('/processed_image', Image, queue_size=1)
    js_pub = rospy.Publisher('/js_image', UInt8MultiArray, queue_size=1)
    
    rospy.init_node('publish_test_image', anonymous=True)

    #bridge = CvBridge()

    image = cv2.imread(__home_path + "/images/box.png")
    image = cv2.resize(image, (640/2, 480/2))
    
    rate = rospy.Rate(10) # 10hz
    js_msg = UInt8MultiArray()
    while not rospy.is_shutdown():
        #pub.publish( bridge.cv2_to_imgmsg(image, "bgr8") )

        js_msg.data = convert_to_js_image_data(image)
        js_pub.publish( js_msg )
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass