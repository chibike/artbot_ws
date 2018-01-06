//!/usr/bin/env node
'use strict';

class ROS_Bridge
{
	constructor()
	{
		this.img = undefined;
		this.ros_int = undefined;
		this.rosnodejs = undefined;
		this.threshold_pub = undefined;

		this.rosnodejs = require('../node_modules/rosnodejs/dist/index.js');
		this.rosnodejs.initNode('/user_interface/nodejs', { onTheFly: true }).then(this.main);

		this.image_update_callback_functions = new Array();
	}

	image_callback(data)
	{
		console.log('image', data);
		for (var i=0; i<this.image_update_callback_functions.length; i++)
		{
			var foo = this.image_update_callback_functions[i];
			foo(data);
		}
	}

	publish_threshold(value)
	{
		if (this.threshold_pub && this.ros_int)
		{
			var msg = new this.ros_int({data: value});
			this.threshold_pub.publish(msg);
		}
	}

	get_image()
	{
		console.log("Fetching image...");
		return this.img;
	}

	add_to_image_update_callback_list(function_name)
	{
		this.image_update_callback_functions.push(function_name);
	}

	main(ros_node)
	{
		// create subscriber for processed image
		var bar = { queueSize: 1, throttleMs: 10};
		ros_node.subscribe('/processed_image1', 'sensor_msgs/Image', function(data){console.log("image",data)}, bar);

		// create publishers for user_interface data
		//var bar = { queueSize: 10, latching: true, throttleMs: 1};
		//this.threshold_pub = ros_node.advertise('/user_interface/nodejs/threshold', 'std_msgs/Int32', bar);

		//this.ros_int = this.rosnodejs.require('std_msgs').msg.Int32;
	}
}

module.exports = { ROS_Bridge: ROS_Bridge }