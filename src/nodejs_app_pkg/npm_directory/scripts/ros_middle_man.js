#!/usr/bin/env node

'use strict';
var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');

var img;
var ros_int;
var threshold_pub;

function processed_image_callback(data)
{
	image_callback(data.processed_image);
}

function image_callback(data)
{
	console.log('image', data);
}

function publish_threshold(value)
{
	if (threshold_pub && ros_int)
	{
		var msg = new ros_int({data: value});
		string_pub.publish(msg);
	}
}

function ros_main(ros_node)
{
	// create subscriber for processed image
	var bar = { queueSize: 1, throttleMs: 1};
	ros_node.subscribe('/processed_image', 'image_processing_pkg/ProcessedImage', processed_image_callback, bar);

	// create publishers for user_interface data
	var bar = { queueSize: 10, latching: true, throttleMs: 1};
	threshold_pub = ros_node.advertise('/user_interface/nodejs/threshold', 'std_msgs/Int32', bar);

	ros_int = rosnodejs.require('std_msgs').msg.Int32;
}

rosnodejs.initNode('/user_interface/nodejs', { onTheFly: true }).then(ros_main);