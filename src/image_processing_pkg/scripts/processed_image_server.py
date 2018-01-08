#!/usr/bin/env python
import os
import time
import rospy
import thread
import cStringIO
import Image as py_image
from sensor_msgs.msg import Image
from flask import Flask, Response, request, abort, render_template_string, send_from_directory

__home_path = "/home/odroid/artbot_ws/src/image_processing_pkg";
processed_image = None

class ProcessedImageServer(object):
    """docstring for ProcessedImageServer"""
    def __init__(self, name, host="localhost", port="5000"):
        super(ProcessedImageServer, self).__init__()
        
        self.app = Flask(name)
        self.address = address
        self.port = port

        self.bridge = CvBridge()
        self.processed_image = None

    def callback(self, data):
        self.processed_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def start_listener(self):
        rospy.init_node('processed_image_server', anonymous=False)
        rospy.Subscriber("processed_image", Image, self.callback)

    @self.app.route("/")
    def test():
        return "Hello!"

    @self.app.route("/image")
    def get_image():
        if self.processed_image:
            rospy.loginfo("Fetching processed image")
            img_str = cv2.imencode('.jpg', self.processed_image)[1].tostring()
            return Response(img_str, mimetype='image/jpeg')
        else:
            rospy.loginfo("Fetching default image")
            im = py_image.open(__home_path + "/images/marguerite-daisy-beautiful-beauty.jpg")
            io = cStringIO.StringIO()
            im.save(io, format='JPEG')
            return Response(io.getvalue(), mimetype='image/jpeg')

    def run():
        self.start_listener()
        thread.start_new_thread( rospy.spin, () )
        self.app.run(host=MY_IP, port=5001)

MY_IP = os.getenv("MY_IP", "10.14.121.64")
if __name__ == '__main__':
    server = ProcessedImageServer(__name__, MY_IP, 5000)
    server.run()