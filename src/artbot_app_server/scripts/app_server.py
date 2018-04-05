#!/usr/bin/env python

import os
import sys
import cv2
import time
import rospy
import serial
import thread
import cStringIO
import actionlib
import Image as py_image
import image_processing_pkg.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from flask import Flask, Response, request, abort, render_template, render_template_string, send_from_directory

PACKAGE_NAME = "artbot_app_server"
PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
PACKAGE_PATH = PACKAGE_PATH[0:PACKAGE_PATH.find(PACKAGE_NAME)] + PACKAGE_NAME + "/"

WEB_FILES_PATH = PACKAGE_PATH + 'web_files/'
HTML_PATH = WEB_FILES_PATH + 'html/'
CSS_PATH = WEB_FILES_PATH + 'css/'
JS_PATH = WEB_FILES_PATH + 'js/'

host = "0.0.0.0"
port = 4000
# app = Flask(__name__)
app = Flask(__name__, static_url_path='', static_folder=WEB_FILES_PATH, template_folder=WEB_FILES_PATH)

serial_port   = "/dev/serial/by-path/platform-xhci-hcd.3.auto-usb-0:1.1:1.0-port0"
serial_baud   = 115200
serial_device = serial.Serial(serial_port, serial_baud, timeout=1)

bridge = CvBridge()
IMAGE_STREAM_FRAME = None

image_processing_node_client = None


@app.route("/")
def test():
	return "Hello World!"

@app.route("/kill_server")
def shutdown_server():
	rospy.loginfo("Shutting down server")
	try:
		thread.exit()
	except Exception as e:
		rospy.logerror(e)

	serial_device.close()
	sys.exit(0)

@app.route("/home")
def home():
	return render_template('html/home.html', title='home')

@app.route("/take_selfie")
def take_selfie():
	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_render_preview");
	image_processing_node_client.send_goal(goal)

	return render_template('html/take_selfie.html', title='take_selfie')

@app.route("/capture_image")
def capture_image():
	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_capture_image");
	image_processing_node_client.send_goal(goal)

	return ""

@app.route("/entry")
def entry():
	return render_template('html/entry.html', title='entry')

@app.route("/favicon.ico")
def favicon():
	return app.send_static_file("images/artbot-logo.png")

@app.route("/input")
def external_input():
	data = serial_device.read(serial_device.in_waiting)
	data = data[0:data.rfind('\n')]
	data = data[data.rfind('\n') + 1:]
	data = data.replace('\r', '')
	return data

@app.route("/get_image_stream")
def get_image_stream():
	global IMAGE_STREAM_FRAME

	if not (type(IMAGE_STREAM_FRAME) == type(None)):
		img_str = cv2.imencode('.jpg', IMAGE_STREAM_FRAME)[1].tostring()
		return Response(img_str, mimetype='image/jpeg')
	else:
		im = py_image.open(WEB_FILES_PATH + "/images/entry.jpg")
		io = cStringIO.StringIO()
		im.save(io, format='JPEG')
		return Response(io.getvalue(), mimetype='image/jpeg')

@app.route("/<path:path>")
def static_file():
	return app.send_static_file(path)

def image_stream_callback(data):
	global IMAGE_STREAM_FRAME
	IMAGE_STREAM_FRAME = bridge.imgmsg_to_cv2(data, 'bgr8')


rospy.loginfo("Initializing node artbot_app_server")
rospy.init_node('artbot_app_server', anonymous=False)

rospy.loginfo("Subscribing to processed_image")
rospy.Subscriber("processed_image", Image, image_stream_callback)

rospy.loginfo("Initializing image_processing_node_client")
image_processing_node_client = actionlib.SimpleActionClient('image_processing_node', image_processing_pkg.msg.StateChangeRequestAction)

rospy.loginfo("image_processing_node_client:: waiting for server")
image_processing_node_client.wait_for_server()

goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_normal");
image_processing_node_client.send_goal(goal)
rospy.loginfo("image_processing_node_client:: connected")

rospy.loginfo("Starting app server")
thread.start_new_thread(app.run, (host, port))

rospy.loginfo("Starting ros spin")
thread.start_new_thread(rospy.spin, ())

# app.run(host=host, port=port)

time.sleep(5)
rospy.loginfo("App Server is now up")

while not rospy.is_shutdown():
	time.sleep(10)

rospy.loginfo("Shutting down!")











