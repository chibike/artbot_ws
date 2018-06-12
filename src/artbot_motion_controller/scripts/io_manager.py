#!/usr/bin/env python

import os
import sys
import time
import serial

#sudo chown ros /dev/ttyUSB0

class DemoSerialDevice(object):
	def __init__(self, port="", baudrate=115200, timeout=0.5):
		self.port = port
		self.baudrate = baudrate
		self.timeout = timeout

		self.buffer = "[100.0]"
		self.in_waiting = len(self.buffer)

	def write(self, data=""):
		pass

	def read(self, n=5):
		return self.buffer

	def close(self):
		pass


class IOManager(object):
	def __init__(self, port="/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0", baudrate=115200):
		self.port = port
		self.baudrate = baudrate
		self.serial_device = None
		self.pressure = 100

	def start(self):
		try:
			self.serial_device = serial.Serial(self.port, self.baudrate, timeout=0.5)
		except Exception as e:
			print "ERROR (IOManager): Could not open serial device"
			print "(IOManager): Starting demo serial device"

			self.serial_device = DemoSerialDevice(self.port, self.baudrate, timeout=0.5)

	def stop(self):
		if not self.serial_device is None:
			self.serial_device.close()

		self.serial_device = None

	def open_gripper(self):
		self.serial_device.write("o")

	def close_gripper(self):
		self.serial_device.write("c")

	def get_pressure(self):
		data = self.serial_device.read(self.serial_device.in_waiting)
		data = data[:data.rfind(']')]
		data = data[data.rfind('[')+1:]
		
		try:
			foo = float(data)

			if foo >= 0 and foo <= 100:
				self.pressure = foo
			else:
				print "HUGE ERROR IN DATA ---------_______", foo, " VS ", data, "!"
		except:
			pass
		
		return self.pressure

def main():
	foo = IOManager()
	foo.start()
	try:
		while True:
			cmd = raw_input("Enter your cmd? ").lower()
			if cmd == "open":
				foo.open_gripper()
			elif cmd == "close":
				foo.close_gripper()

			print "pressure =", foo.get_pressure()
	except KeyboardInterrupt:
		print "Exiting...."
	foo.stop()



if __name__ == '__main__':
	main()