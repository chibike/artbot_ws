//!/usr/bin/env node
'use strict';

var rosnodejs = require('../node_modules/rosnodejs/dist/index.js');

class ROS_Bridge
{
	constructor()
	{
		this.img = undefined;
		this.ros_int = undefined;
		this.threshold_pub = undefined;

		this.image_update_callback_functions = new Array();

		var outer_this = this;

		rosnodejs.initNode('/user_interface/nodejs', { onTheFly: true }).then(
				function (ros_node)
				{
					// create subscriber for processed image
					var bar = { queueSize: 10, throttleMs: 1000};
					console.log("subscribing...");
					
					var foo = ros_node.subscribe('/processed_image', 'sensor_msgs/Image', function(data)
						{
							this.img = data.data;
							var img_height = data.height;
							var img_width = data.width;
						},
						bar
					);

					//console.log("result", foo);
					console.log("Done");

					// create publishers for user_interface data
					var bar = { queueSize: 1, latching: true, throttleMs: 10};
					outer_this.threshold_pub = ros_node.advertise('/user_interface/nodejs/threshold', 'std_msgs/Int32', bar);

					outer_this.ros_int = rosnodejs.require('std_msgs').msg.Int32;
				}
			);
	}

	publish_threshold(value)
	{
		if (this.threshold_pub && this.ros_int)
		{
			var msg = new ros_int({data: value});
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
		
	}
}

//ROS_Bridge.image_callback
module.exports = { ROS_Bridge: ROS_Bridge }