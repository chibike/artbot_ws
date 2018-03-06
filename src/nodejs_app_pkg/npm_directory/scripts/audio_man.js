//!/usr/bin/env node
'use strict';

class Audio_Man
{
	constructor()
	{
		this.spawn = require('child_process').spawn
		this.find_exec = require('find-exec')
		this.random_number = require('random-number')
		
		this.player = this.find_exec(['mplayer', 'afplay', 'mpg123', 'mpg321', 'play', 'omxplayer', 'aplay', 'cmdmp3']);
		if (!this.player)
		{
			console.log("Couldn't find a suitable audio player");
		}
		
		this.music_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/music/";
		this.audio_files = ["music_1.mp3", "music_2.mp3"];

		this.speech_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/speech/";
		this.speech_files = ["hey_you_by_cousin_edit.wav"];
		
		this.speech = undefined;
		this.audio = undefined;

		var options = {
			min: 0,
			max: this.audio_files.length-1,
			integer: true
		}
		this.current_index = this.random_number(options);
	}

	start()
	{
		if (!this.audio)
		{
			this.stop_audio();
			var audio_file = this.music_directory + this.audio_files[this.current_index];
			this.play(audio_file);
		}

		this.increase_volume(50);
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
			if (code == '0')
			{
				outer_this.next();
			}
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
			console.log(context, "Unable to spawn process with " + this.player);
			return;
		}

		this.audio.stdin.setEncoding('utf-8');

		this.audio.on('message', on_message);
		this.audio.on('close', on_close);
		this.audio.on('error', on_error);
		this.audio.on('exit', on_exit);


		console.log("playing :", audio_file);
	}

	play_speech(index)
	{
		index = index % this.speech_files.length;
		var audio_file = this.speech_directory + this.speech_files[index];

		var outer_this = this;
		var context = "speech";

		var on_message = function(message, send_handle)
		{
			console.log(context, 'message', message, send_handle);
		}

		var on_close = function(code, signal)
		{
			console.log(context, 'close', code, signal);
			if (code == '0')
			{
				outer_this.toggle_pause();
			}
		}

		var on_exit = function(code, string)
		{
			console.log(context, 'exit', code, string);
		}

		var on_error = function(err)
		{
			console.log(context, 'error', err);
		}

		this.toggle_pause();

		this.speech = this.spawn(this.player, [audio_file], {});
		if (!this.speech)
		{
			console.log(context, "Unable to spawn process with " + this.player);
			return;
		}

		this.speech.stdin.setEncoding('utf-8');

		this.speech.on('message', on_message);
		this.speech.on('close', on_close);
		this.speech.on('error', on_error);
		this.speech.on('exit', on_exit);

		console.log("saying :", audio_file);
	}

	next()
	{
		this.stop_audio();

		this.current_index = (this.current_index + 1) % this.audio_files.length;
		var audio_file = this.music_directory + this.audio_files[this.current_index];
		
		this.play(audio_file);
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

	toggle_pause()
	{
		if (this.audio && !this.audio.killed)
		{
			this.audio.stdin.write("p");
			// this.audio.stdin.end();
		}
	}

	toggle_mute()
	{
		if (this.audio && !this.audio.killed)
		{
			this.audio.stdin.write("m");
			// this.audio.stdin.end();
		}
	}

	decrease_volume(amount)
	{
		if (this.audio && !this.audio.killed)
		{
			for (var i=0; i<amount; i++)
			{
				this.audio.stdin.write("9");
			}
		}
	}

	increase_volume(amount)
	{
		if (this.audio && !this.audio.killed)
		{
			for (var i=0; i<amount; i++)
			{
				this.audio.stdin.write("0");
			}
		}
	}
}

//ROS_Bridge.image_callback
module.exports = { Audio_Man: Audio_Man }