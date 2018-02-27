//!/usr/bin/env node
'use strict';

class Audio_Man
{
	constructor()
	{
		this.player = require('play-sound')();//(opts = {mplayer});
		this.music_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/music/";
		this.audio_files = ["ukulele.mp3", "breeze.mp3", "buddy.mp3", "happyrock.mp3", "himitsu.mp3"];

		this.speech_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/speech/";
		this.speech_files = ["test_speech.wav"];
		
		this.speech = undefined;
		this.audio = undefined;
		this.current_index = 0;
	}

	start()
	{
		this.stop();
		var audio_file = this.music_directory + this.audio_files[this.current_index];
		this.play(audio_file);
	}

	stop()
	{
		if (this.audio)
		{
			this.audio.kill();
		}
	}

	stop_speech()
	{
		if (this.speech)
		{
			this.speech.kill();
		}
	}

	stop_all()
	{
		this.stop();
		this.stop_speech();
	}


	play_speech(index)
	{
		index = index % this.speech_files.length;
		var audio_file = this.speech_directory + this.speech_files[index];
		
		this.speech = this.player.play(audio_file, this.error_handler);
	}

	play(audio_file)
	{
		this.audio = this.player.play(audio_file, this.error_handler);
		console.log(this.audio);
	}

	pause()
	{
	}

	continue()
	{
	}

	next()
	{
		this.stop();
		this.current_index = (this.current_index + 1) % this.audio_files.length;
		var audio_file = this.music_directory + this.audio_files[this.current_index];
		this.play(audio_file);
	}

	error_handler(err)
	{
		console.log("audio_error", err);
		// if (err && !err.killed) throw err
	}
}

//ROS_Bridge.image_callback
module.exports = { Audio_Man: Audio_Man }