//!/usr/bin/env node
'use strict';

class Audio_Man
{
	constructor()
	{
		this.player = require('play-sound')();//(opts = {mplayer});
		this.music_directory = "/home/odroid/artbot_ws/src/nodejs_app_pkg/npm_directory/music/";
		this.audio_files = ["himitsu.mp3"];
		
		this.audio = undefined;
		this.current_index = undefined;
	}

	start()
	{
		this.stop();
		var audio_file = this.music_directory + this.audio_files[this.current_index];
		this.audio = this.player.play(audio_file, this.error_handler);
	}

	stop()
	{
		if (this.audio)
		{
			this.audio.kill();
		}

		this.current_index = 0;
	}

	pause()
	{

	}

	continue()
	{

	}

	error_handler(err)
	{
		console.log("audio_error", err);
		if (err && !err.killed) throw err
	}
}

//ROS_Bridge.image_callback
module.exports = { Audio_Man: Audio_Man }