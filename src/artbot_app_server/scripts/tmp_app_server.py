#!/usr/bin/env python
import os
import sys
import cv2
import time
import rospy
import serial
import thread
import cStringIO
import Image as py_image
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


class AppServer(object):
    app = None

    def __init__(self, name):
        self.host = "0.0.0.0"
        self.port = 6000
        self.app = Flask(name, static_url_path='', static_folder=WEB_FILES_PATH, template_folder=WEB_FILES_PATH)
        # self.app.debug = True
        # self.app.secret_key = 'artbot key'

        self.serial_port = "/dev/serial/by-path/platform-xhci-hcd.3.auto-usb-0:1.1:1.0-port0"
        self.serial_baud = 115200
        self.serial_device = serial.Serial(self.serial_port, self.serial_baud, timeout=1)

        self.bridge = CvBridge()
        self.image_stream_frame = None

        self.app.route("/")(self.test)
        self.app.route("/kill_server")(self.shutdown_server)
        self.app.route("/home")(self.home)
        self.app.route("/entry")(self.entry)
        self.app.route("/favicon.ico")(self.favicon)
        self.app.route("/input")(self.external_input)
        self.app.route("/get_image_stream")(self.get_image_stream)

        self.app.route("/<path:path>")(self.static_file)

    def shutdown_server(self):
        rospy.loginfo("Shutting Down")
        thread.exit()
        sys.exit(0)

    def image_stream_callback(self, data):
        self.image_stream_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def get_image_stream(self):
        if not (type(self.image_stream_frame) == type(None)):
            img_str = cv2.imencode('.jpg', self.image_stream_frame)[1].tostring()
            return Response(img_str, mimetype='image/jpeg')
        else:
            im = py_image.open(WEB_FILES_PATH + "/images/entry.jpg")
            io = cStringIO.StringIO()
            im.save(io, format='JPEG')
            return Response(io.getvalue(), mimetype='image/jpeg')

    def start_image_stream_listener(self):
        rospy.init_node('artbot_app_server', anonymous=False)
        rospy.Subscriber("processed_image", Image, self.image_stream_callback)

    def external_input(self):
        data = self.serial_device.read(self.serial_device.in_waiting)
        data = data[0:data.rfind('\n')];
        data = data[data.rfind('\n') + 1:]
        data = data.replace('\r', '')
        return data

    def favicon(self):
        return self.app.send_static_file("images/artbot-logo.png")

    def test(self):
        return "Hello!"

    def advertise(self):
        return "Hello!"

    def explain_interface(self):
        return "Hello!"

    def camera_view(self):
        return "Hello!"

    def home(self):
        return render_template('html/home.html', title='home')

    def entry(self):
        return render_template('html/entry.html', title='entry')

    def static_file(self, path):
        print "static_file ", path
        return self.app.send_static_file(path)

    def run(self):
        # self.start_image_stream_listener()
        # rospy.on_shutdown(self.shutdown_server)

        time.sleep(5)

        # rospy.loginfo("Starting App Server....")
        self.app.run(host=self.host, port=self.port)


def main(name="{0}_app_server".format(__name__)):
    print "In Main"
    server = AppServer(name)
    server.run()


if __name__ == '__main__':
    rospy.loginfo("Running as main")
    main()
else:
    rospy.loginfo("Running as imported")
    main()







