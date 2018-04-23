#!/usr/bin/env python

import os
import sys
import time
import serial


class IOManager(object):
	def __init__(self):
		self.serial_device = None

	def start(self):
		pass

	def stop(self):
		if not self.serial_device is None:
			self.serial_device.close()

		self.serial_device = None

	def open_gripper(self):
		pass

	def close_gripper(self):
		pass

	def get_pressure(self):
		pass




def main():
	foo = IOManager()
	foo.start()
	foo.stop()



if __main__ == 'main':
	main()
		