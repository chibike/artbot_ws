#!/usr/bin/env node

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
	}

	main(ros_node)
	{
		// create subscriber for processed image
		var bar = { queueSize: 1, throttleMs: 1};
		this.ros_node.subscribe('/processed_image', 'image_processing_pkg/ProcessedImage', processed_image_callback, bar);

		// create publishers for user_interface data
		var bar = { queueSize: 10, latching: true, throttleMs: 1};
		this.threshold_pub = this.ros_node.advertise('/user_interface/nodejs/threshold', 'std_msgs/Int32', bar);

		this.ros_int = this.rosnodejs.require('std_msgs').msg.Int32;
	}

	processed_image_callback(data)
	{
		this.image_callback(data.processed_image);
	}

	image_callback(data)
	{
		console.log('image', data);
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
		return this.img;
	}
}

exports.ROS_Bridge = new ROS_Bridge();