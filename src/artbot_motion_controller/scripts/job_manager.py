#!/usr/bin/env python

import sys
import json
import copy
import rospy
from std_msgs.msg import String
from motion_controller import MotionController

class JobManager(object):
	def __init__(self):
		self.jobs = []
		self.motion_controller = MotionController()

		self.page_size = (0.15, 0.20)
		self.page_offset = (self.motion_controller.position_limits["x"][0] + 0.01, -0.5 * self.page_size[1])

		self.x_start = self.page_offset[0]
		self.y_start = self.page_offset[1]
		self.x_end   = self.page_offset[0] + self.page_size[0]
		self.y_end   = self.page_offset[1] + self.page_size[1]

		self.x_scalar = self.page_size[0] / 480.0
		self.y_scalar = self.page_size[1] / 640.0

		self.transform_x = lambda x : (x * self.x_scalar) + self.page_offset[0]
		self.transform_y = lambda y : (y * self.y_scalar) + self.page_offset[1]

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
			self.motion_controller.stop()
		except:
			pass

	def shutdown(self):
		self.stop()

	def callback(self, data):
		self.job = []

		try:
			self.job = json.loads(data.data)
			with open("/home/ros/artbot_ws/src/artbot_motion_controller/logs/last_path.txt", "w") as f:
				f.write(data.data)
		except Exception as e:
			self.job = []
			with open("/home/ros/artbot_ws/src/artbot_motion_controller/logs/error_path.txt", "w") as f:
				f.write(data.data)
							
		# self.jobs.append(job)
		self.draw_job(self.job)

	def cmd_callback(self, data):
		cmd = data.data;

		if cmd == "calculate_z":
			self.calculate_z()
		elif cmd.startswith("test_file"):
			# test_file: /home/ros/logs.txt
			filename = cmd.lstrip("test_file: ").strip()
			print "*************+++++++++++++++++++ ", cmd, " +++ ", filename

			str_data = ""
			with open(filename, "r") as f:
				str_data = f.read()

			data = String()
			data.data = str_data

			self.callback(data)
		else:
			self.__loginfo("cmd_callback", "Received invalid command \"{cmd}\"".format(cmd=cmd))

	def calculate_z(self):
		# self.motion_controller.calculate_z_depth((0.35, 0.0))
		self.__loginfo("draw_job", "calculating z")
		self.motion_controller.calibrate(self.x_start, self.y_start, self.x_end, self.y_end)
		self.motion_controller.home()

	def break_up(self, paths, max_length=60):
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

	def pre_process(self, paths):
		xs_array = list()
		ys_array = list()
		zs_array = list()

		for path in paths:
			_ys, _xs = list(zip(*path))

			xs = list(map(self.transform_x, _xs));# xs.append(xs[-1])
			ys = list(map(self.transform_y, _ys));# ys.append(ys[-1])
			zs = list([self.motion_controller.pen_writing_position]*len(xs))#; zs.append(self.motion_controller.pen_hover_position)

			xs_array.append(xs)
			ys_array.append(ys)
			zs_array.append(zs)

			nxs = copy.deepcopy(xs); nxs.reverse()
			nys = copy.deepcopy(ys); nys.reverse()
			nzs = copy.deepcopy(zs); nzs.reverse()

			n = 20

			xs_array.append(nxs[:n])
			ys_array.append(nys[:n])
			zs_array.append(nzs[:n])

		return [xs_array,ys_array,zs_array]

	def draw_job(self, job):
		pick_speed  = 1.2
		move_speed  = 1.2
		write_speed = 1.5
		max_trials  =   2

		paths = self.break_up(job)
		number_of_paths = len(paths)
		self.__loginfo("draw_job", "received data as {n} paths".format(n=len(job)))

		self.__loginfo("draw_job", "calculating z")
		self.motion_controller.calibrate(self.x_start, self.y_start, self.x_end, self.y_end)
		
		rospy.loginfo("job_manager:: moving to home position")
		success = False
		for i in range(max_trials):
			if self.motion_controller.home():
				success = True
				break
			else:
				rospy.sleep(0.1)
				continue

		if not success:
			rospy.loginfo("job_manager:: ERROR could not go to home")
			return

		paths = self.pre_process(paths)

		for index in xrange(min([len(paths[0]), len(paths[1]), len(paths[2])])):
			xs = paths[0][index]
			ys = paths[1][index]
			zs = paths[2][index]

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					rospy.sleep(0.1)
					continue

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_writing_position, speed=move_speed):
					break
				else:
					rospy.sleep(0.1)
					continue

			completed = self.motion_controller.follow(xs, ys, zs, speed=write_speed)

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[-1], ys[-1], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					rospy.sleep(0.1)
					continue

			# if not completed:
			# 	rospy.loginfo("job_manager:: Skipping path {path_index}".format(path_index=index))
			# 	continue

			# completed = self.motion_controller.follow_reverse(xs, ys, zs, speed=write_speed)
			# for i in range(max_trials):
			# if self.motion_controller.move_to(xs[-1], ys[-1], self.motion_controller.pen_hover_position, speed=move_speed):
			# 	break
			# else:
			# 	continue

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

def draw_job(self, job):
		pick_speed  = 1.2
		move_speed  =   1
		write_speed =   2
		max_trials  =   1

		paths = self.break_up(job)
		number_of_paths = len(paths)
		self.__loginfo("draw_job", "received data as {n} paths".format(n=len(job)))

		self.__loginfo("draw_job", "calculating z")
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
			_ys, _xs = list(zip(*path))
			xs = list(map(self.transform_x, _xs))
			ys = list(map(self.transform_y, _ys))
			zs = list([self.motion_controller.pen_writing_position]*len(xs))

			xs.append(xs[-1])
			ys.append(ys[-1])
			zs.append(self.motion_controller.pen_hover_position)

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					continue

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_writing_position, speed=move_speed):
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
'''
