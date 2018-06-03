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
from music_server import AudioManager
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
app = Flask(__name__, static_url_path='', static_folder=WEB_FILES_PATH, template_folder=WEB_FILES_PATH)

serial_port   = "/dev/serial/by-path/platform-xhci-hcd.3.auto-usb-0:1.1:1.0-port0"
serial_baud   = 115200
serial_device = serial.Serial(serial_port, serial_baud, timeout=1)

bridge = CvBridge()
IMAGE_STREAM_FRAME = None

image_processing_node_client = None

MUSIC_DIR  = "{0}music/".format(PACKAGE_PATH)
SPEECH_DIR = "{0}speech/".format(PACKAGE_PATH)

audio_manager = AudioManager()
audio_manager.add_song_directory(MUSIC_DIR)
audio_manager.add_song_directory("/media/odroid/ARTBOT_BP/music/")
audio_manager.add_speech('hey_you', SPEECH_DIR + 'hey_you_by_cousin_edit.wav')
audio_manager.add_speech('how_to_take_a_picture', SPEECH_DIR + 'how_to_take_a_picture.wav')
audio_manager.add_speech('selfie_mode', SPEECH_DIR + 'selfie_mode.wav')
audio_manager.add_speech('thank_you', SPEECH_DIR + 'thank_you.wav')
audio_manager.add_speech('image_is_being_processed', SPEECH_DIR + 'your_image_is_being_processed.wav')


@app.route("/")
def test():
	return "Hello World!"

@app.route("/kill_server")
def shutdown_server():
	rospy.loginfo("Shutting down server")
	try:
		rospy.loginfo("audio-manager: Shutting the audio server....")
		audio_manager.stop()
		# rospy.loginfo("audio-manager: Waiting for server to shutdown....")
		# time.sleep(3)
		rospy.loginfo("audio-manager: Done")
	except :
		rospy.logerror("audio-manager: shutdown failed")

	try:
		rospy.loginfo("serial-port: closing...")
		serial_device.close()
		rospy.loginfo("serial-port: closed")
	except :
		rospy.logerror("serial-port: close failed")

	time.sleep(3)

	try:
		rospy.loginfo("app-server: Killing thread")
		thread.exit()

		# rospy doesn't exist because it's thread has been killed
		print("app-server: Done")
	except :
		print("audio-manager: shutdown failed")

	sys.exit(0)

@app.route("/exit")
def app_exit():
	audio_manager.say('thank_you')
	rospy.signal_shutdown("App requests shutdown")
	return ""


@app.route("/home")
def home():
	return render_template('html/home.html', title='home')

@app.route("/settings")
def settings():
	return render_template('html/settings.html', title='settings')

@app.route("/take_selfie")
def take_selfie():
	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_render_preview");
	image_processing_node_client.send_goal(goal)
	audio_manager.say('how_to_take_a_picture')
	
	return render_template('html/take_selfie.html', title='take_selfie')

@app.route("/capture_image")
def capture_image():
	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_capture_image");
	image_processing_node_client.send_goal(goal)

	audio_manager.say('image_is_being_processed')

	return ""

@app.route("/stop_image_stream")
def stop_image_stream():
	goal = image_processing_pkg.msg.StateChangeRequestGoal(state="state_normal");
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

rospy.on_shutdown(shutdown_server)

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

time.sleep(5)
rospy.loginfo("App Server is now up")

rospy.loginfo("Starting audio server")
thread.start_new_thread(audio_manager.start, ())

rospy.loginfo("Starting ros spin")
rospy.spin()
# thread.start_new_thread(rospy.spin, ())

# while not rospy.is_shutdown():
# 	time.sleep(10)

rospy.loginfo("Shutting down!")











