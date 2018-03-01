//!/usr/bin/env node
'use strict';

class Audio_Man
{
	constructor()
	{
		this.spawn = require('child_process').spawn
		this.find_exec = require('find-exec')
		
		this.player = this.find_exec(['mplayer', 'afplay', 'mpg123', 'mpg321', 'play', 'omxplayer', 'aplay', 'cmdmp3']);
		if (!this.player)
		{
			console.log("Couldn't find a suitable audio player");
		}
		
		this.music_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/music/";
		this.audio_files = ["breeze.mp3", "buddy.mp3", "happyrock.mp3", "himitsu.mp3", "ukulele.mp3"];

		this.speech_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/speech/";
		this.speech_files = ["test_speech.wav"];
		
		this.speech = undefined;
		this.audio = undefined;
		this.current_index = 0;
	}

	start()
	{
		if (!this.audio)
		{
			this.stop_audio();
			var audio_file = this.music_directory + this.audio_files[this.current_index];
			this.play(audio_file);
		}
	}

	play(audio_file)
	{
		var outer_this = this;
		var context = "audio";

		var on_message = function(message, send_handle)
		{
			console.log(context, 'message', message, send_handle);
		}

		var on_close = function(code, signal)
		{
			console.log(context, 'close', code, signal);
			outer_this.next();
		}

		var on_exit = function(code, string)
		{
			console.log(context, 'exit', code, string);
		}

		var on_error = function(err)
		{
			console.log(context, 'error', err);
		}

		// this.audio = this.spawn(this.player, [audio_file], {stdio: 'ignore'});
		this.audio = this.spawn(this.player, [audio_file], {});
		if (!this.audio)
		{
			console.log("Unable to spawn process with " + this.player);
			return;
		}

		this.audio.stdin.setEncoding('utf-8');

		this.audio.on('message', on_message);
		this.audio.on('close', on_close);
		this.audio.on('error', on_error);
		this.audio.on('exit', on_exit);
	}

	next()
	{
		this.stop_audio();

		this.current_index = (this.current_index + 1) % this.audio_files.length;
		var audio_file = this.music_directory + this.audio_files[this.current_index];
		
		this.play(audio_file);
	}

	toggle_pause()
	{
		if (this.audio && !this.audio.killed)
		{
			this.audio.stdin.write("p");
			// this.audio.stdin.end();
		}
	}

	stop_audio()
	{
		if (this.audio && !this.audio.killed)
		{
			this.audio.kill();
		}
	}

	stop_speech()
	{
		if (this.speech && !this.speech.killed)
		{
			this.speech.kill();
		}
	}

	stop_all()
	{
		this.stop_audio();
		this.stop_speech();
	}


	play_speech(index)
	{
		index = index % this.speech_files.length;
		var audio_file = this.speech_directory + this.speech_files[index];
		
		this.speech = this.player.play(audio_file, this.error_handler);
	}
}

//ROS_Bridge.image_callback
module.exports = { Audio_Man: Audio_Man }