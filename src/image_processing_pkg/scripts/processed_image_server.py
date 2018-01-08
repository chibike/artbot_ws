#!/usr/bin/env python
import os
import rospy
import cStringIO
import Image as py_image
from sensor_msgs.msg import Image
from flask import Flask, Response, request, abort, render_template_string, send_from_directory

__home_path = "/home/odroid/artbot_ws/src/image_processing_pkg";
image = None

def callback(data):
    image = data.data
    
def start_listener():
    rospy.init_node('processed_image_server', anonymous=False)
    rospy.Subscriber("processed_image", Image, callback)

app = Flask(__name__)
#MY_IP = os.getenv("MY_IP", "localhost")
MY_IP = os.getenv("MY_IP", "10.14.122.58")

@app.route("/")
def hello():
    return "Hello!"

@app.route("/image")
def get_image():
    im = Image.open(__home_path + "/images/cover_image.jpg")
    io = cStringIO.StringIO()
    im.save(io, format='JPEG')
    return Response(io.getvalue(), mimetype='image/jpeg')

if __name__ == '__main__':
    start_listener()
    app.run(host=MY_IP)