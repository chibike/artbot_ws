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
image = None

def callback(data):
    image = data.data
    rospy.loginfo("Hello")
    
def start_listener():
    rospy.init_node('processed_image_server', anonymous=False)
    rospy.Subscriber("processed_image", Image, callback)

def run():
    rospy.spin()

app = Flask(__name__)
#MY_IP = os.getenv("MY_IP", "localhost")
MY_IP = os.getenv("MY_IP", "localhost")

@app.route("/")
def hello():
    return "Hello!"

@app.route("/image")
def get_image():
    im = py_image.open(__home_path + "/images/box.png")
    io = cStringIO.StringIO()
    im.save(io, format='JPEG')
    return Response(io.getvalue(), mimetype='image/jpeg')

if __name__ == '__main__':
    start_listener()
    thread.start_new_thread( run() )
    app.run(host=MY_IP, port=5001)