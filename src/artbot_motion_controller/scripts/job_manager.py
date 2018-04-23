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

		self.page_length = (0.165, 0.220)
		self.page_offset = (0.28, -0.5 * self.page_length[1])

		self.transform_row_to_x = lambda row : (row * self.page_length[0] / 480.0) + self.page_offset[0]
		self.transform_col_to_y = lambda col : (col * self.page_length[1] / 640.0) + self.page_offset[1]

	def start(self):
		self.motion_controller.start()
		rospy.Subscriber("path_as_string", String, self.callback)

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
		# self.jobs.append(job)
		self.draw_job(self.job)

	def break_up(self, paths, max_length=20):
		def _break_up(path):
			new_paths = []
			while len(path) > max_length:
				new_path = path[:max_length]
				new_paths.append(new_path)
				path = path[max_length:]

			return new_paths

		new_paths = []
		for path in paths:
			new_paths += _break_up(path)

		return new_paths

	def change_pen(self, color):
		rospy.loginfo("job_manager:: changing pen")

	def draw_job(self, job):
		pick_speed  = 1.2
		move_speed  =   4
		write_speed =   3
		max_trials  =   2

		paths = self.break_up(job)
		number_of_paths = len(paths)
		rospy.loginfo("job_manager:: received data as {n} paths".format(n=number_of_paths))
		
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
			zs = list([self.motion_controller.pen_writing_position]*len(xs))

			xs.append(xs[-1])
			ys.append(ys[-1])
			zs.append(self.motion_controller.pen_writing_position+0.03)

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_hover_position, speed=move_speed):
					break
				else:
					continue

			for i in range(max_trials):
				if self.motion_controller.move_to(xs[0], ys[0], self.motion_controller.pen_writing_position+0.03, speed=pick_speed):
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
