#!/usr/bin/env python

import sys
import json
import rospy
import thread
from std_msgs.msg import String
from motion_controller import MotionController

class JobManager(object):
	def __init__(self):
		self.jobs = []
		self.motion_controller = MotionController()

		self.page_length = (0.15, 0.20)
		self.page_offset = (0.26, -0.5 * self.page_length[1])

		self.x_start = self.page_offset[0]
		self.y_start = self.page_offset[1]
		self.x_end   = self.page_offset[0] + self.page_length[0]
		self.y_end   = self.page_offset[1] + self.page_length[1]

		self.transform_row_to_x = lambda row : (row * self.page_length[0] / 480.0) + self.page_offset[0]
		self.transform_col_to_y = lambda col : (col * self.page_length[1] / 640.0) + self.page_offset[1]

	def __loginfo(self, name="", info=""):
		rospy.loginfo("JobManager::{name} -> {info}".format(name=name, info=info))

	def __logerror(self, name, info):
		rospy.logerror("JobManager::{name} -> {info}".format(name=name, info=info))

	def start(self):
		self.motion_controller.start()
		rospy.Subscriber("path_as_string", String, self.callback)
		rospy.Subscriber("job_manager_cmd", String, self.cmd_callback)

	def stop(self):
		try:
			thread.exit()
		except:
			pass

		try:
			self.motion_controller.stop()
		except:
			pass

	def shutdown(self):
		self.stop()

	def callback(self, data):
		self.job = json.loads(data.data)
		with open("/home/ros/logs.txt", "w") as f:
			f.write(data.data)
			self.__loginfo("callback", "logged paths")
			
		# self.jobs.append(job)
		self.draw_job(self.job)

	def cmd_callback(self, data):
		cmd = data.data;

		if cmd == "change_pen":
			#self.change_pen()
			pass
		elif cmd == "calculate_z":
			self.calculate_z()
		else:
			self.__loginfo("cmd_callback", "Received invalid command \"{cmd}\"".format(cmd=cmd))

	def change_pen(self):
		self.__loginfo("change_pen", "Homing")
		self.motion_controller.home()

		self.__loginfo("change_pen", "Changing pen")
		self.motion_controller.change_pen((0.35, 0.155), (0.35, 0.155))

		self.__loginfo("change_pen", "Homing")
		self.motion_controller.home()

	def calculate_z(self):
		# self.motion_controller.calculate_z_depth((0.35, 0.0))
		self.motion_controller.calibrate(self.x_start, self.y_start, self.x_end, self.y_end)
		self.motion_controller.home()

	def break_up(self, paths, max_length=50):
		def _break_up(path):
			new_paths = []
			while len(path) > max_length:
				new_path = path[:max_length]
				new_paths.append(new_path)
				path = path[max_length:]

			if len(path) > 0: new_paths.append(path)
			return new_paths

		new_paths = []
		for path in paths:
			new_paths += _break_up(path)

		return new_paths

	def draw_job(self, job):
		pick_speed  = 1.2
		move_speed  =   3
		write_speed = 1.5
		max_trials  =   1

		paths = self.break_up(job)
		number_of_paths = len(paths)
		self.__loginfo("draw_job", "received data as {n} paths".format(n=len(job)))

		self.__loginfo("draw_job", "calculating z")
		# self.motion_controller.calculate_z_depth((0.35, 0.0))
		self.motion_controller.calibrate(self.x_start, self.y_start, self.x_end, self.y_end)
		
		rospy.loginfo("job_manager:: moving to home position")
		success = False
		for i in range(max_trials):
			if self.motion_controller.home():
				success = True
				break
			else:
				continue

		if not success:
			rospy.loginfo("job_manager:: ERROR could not go to home")
			return

		for index, path in enumerate(paths):
			rows, cols = list(zip(*path))
			xs = list(map(self.transform_row_to_x, rows))
			ys = list(map(self.transform_col_to_y, cols))
			zs = []

			for i in range(min(len(xs), len(ys))):
				zs.append(self.motion_controller.get_z_depth(xs[i], ys[i], self.x_start, self.y_start, self.x_end, self.y_end, self.motion_controller.pressure_constants))

			zs = list([self.motion_controller.pen_writing_position]*len(xs))

			xs.append(xs[-1])
			ys.append(ys[-1])
			zs.append(self.motion_controller.pen_writing_position+0.03)

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					continue

			completed = self.motion_controller.follow(xs, ys, zs, speed=write_speed)

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[-1], ys[-1], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					continue

			if not completed:
				rospy.loginfo("job_manager:: Skipping path {path_index}".format(path_index=index))
				continue

		rospy.loginfo("job_manager:: moving to home position")
		success = False
		for i in range(max_trials):
			if self.motion_controller.home():
				success = True
				break
			else:
				continue

		if not success:
			rospy.loginfo("job_manager:: ERROR could not go to home")
			return

	def run(self):
		pass

		

def main():
	job_manager = JobManager()
	job_manager.start()

	rospy.on_shutdown(job_manager.shutdown)
	rospy.spin()

if __name__ == '__main__':
	main()

'''
introduction and research
'''
