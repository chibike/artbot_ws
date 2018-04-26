#!/usr/bin/env python

import os
import sys
import time
import thread
import pygame
import random

PACKAGE_NAME = "artbot_app_server"
PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
PACKAGE_PATH = PACKAGE_PATH[0:PACKAGE_PATH.find(PACKAGE_NAME)] + PACKAGE_NAME + "/"

MUSIC_DIR  = "{0}music/".format(PACKAGE_PATH)
SPEECH_DIR = "{0}speech/".format(PACKAGE_PATH)

print "MUSIC_DIR =", MUSIC_DIR
print "SPEECH_DIR =", SPEECH_DIR

pygame.init()

class AudioManager(object):
	def __init__(self, bg_volume=0.15, fg_volume=0.95):
		self.songs = []
		self.speeches = dict()

		self.song_index = -1
		self.bg_volume = bg_volume
		self.fg_volume = fg_volume

		self.MUSIC_END = pygame.USEREVENT + 1
		self.kill = False

		self.is_paused = False

		pygame.mixer.music.set_endevent(self.MUSIC_END)
		pygame.mixer.music.set_volume(self.bg_volume)

	def add_song_directory(self, directory):
		self.songs += [directory + i for i in os.listdir(directory)]
		return True

	def add_song(self, directory):
		self.songs.append(directory)
		return True

	def add_speech(self, tag, directory):
		if tag in self.speeches.keys():
			return False
		
		try:
			self.speeches[tag] = pygame.mixer.Sound(directory)
			self.speeches[tag].set_volume(self.fg_volume)
		except Exception as e:
			return False

		return True

	def start(self):
		self.song_index = random.randint(0, len(self.songs)-1)
		pygame.mixer.music.load( self.songs[self.song_index] )
		pygame.mixer.music.play()

		clock = pygame.time.Clock()

		while not self.kill:
			for event in pygame.event.get():
				if event.type == self.MUSIC_END:
					self.on_music_end()
			clock.tick(3)

	def toggle_pause(self):
		if not self.is_paused:
			pygame.mixer.music.pause()
		else:
			pygame.mixer.music.unpause()

		self.is_paused = not self.is_paused

	def stop(self, time_ms=500):
		self.kill = True
		pygame.mixer.music.fadeout(time_ms)
		time.sleep(time_ms/1000)
		pygame.quit()

	def say(self, tag):
		if not tag in self.speeches.keys():
			return False

		try:
			self.speeches[tag].play()
		except Exception as e:
			return False

		return True

	def on_music_end(self):
		self.song_index += 1
		self.song_index = self.song_index % len(self.songs)

		print "Starting next song...."
		pygame.mixer.music.load( self.songs[self.song_index] )
		pygame.mixer.music.play()

def main():
	audio_manager = AudioManager()
	audio_manager.add_song_directory(MUSIC_DIR)
	audio_manager.add_speech('hey_you', SPEECH_DIR + 'hey_you_by_cousin_edit.wav')

	print "Starting...."
	thread.start_new_thread(audio_manager.start, ())

	print "use ctrl-c to end"

	try:
		while True:
			time.sleep(random.randint(10, 120))
			audio_manager.say('hey_you')
	except KeyboardInterrupt:
		pass

	time.sleep(10)

	print "Stopping...."
	audio_manager.stop()
	thread.exit()
	sys.exit()


if __name__ == '__main__':
	main()