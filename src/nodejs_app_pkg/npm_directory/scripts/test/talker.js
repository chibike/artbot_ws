#!/usr/bin/env node

'use strict';

var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');

rosnodejs.initNode('/talker_node', { onTheFly: true }).then(function (rosNode) {
  var string_pub = rosNode.advertise('/chatter', 'std_msgs/String', {
    queueSize: 10,
    latching: true,
    throttleMs: 9
  });

  var string = rosnodejs.require('std_msgs').msg.String;
  var msg_string = new string({
    data: "Hello, world"
  });
  string_pub.publish(msg_string);

});