#!/usr/bin/env python

import os
import sys
import time
import serial


class IOManager(object):
	def __init__(self, port="", baudrate=115200):
		self.port = port
		self.baudrate = baudrate
		self.serial_device = None

	def start(self):
		self.serial_device = serial.Serial(self.port, self.baudrate, timeout=0.5)

	def stop(self):
		if not self.serial_device is None:
			self.serial_device.close()

		self.serial_device = None

	def open_gripper(self):
		self.serial_device.write("O")

	def close_gripper(self):
		self.serial_device.write("C")

	def get_pressure(self):
		data = self.serial_device.read(self.serial_device.in_waiting)
		data = data[0:data.rfind('\n')]
		data = data[data.rfind('\n') + 1:]
		data = data.replace('\r', '')
		return data

def main():
	foo = IOManager()
	foo.start()
	foo.stop()



if __main__ == 'main':
	main()
		