#!/usr/bin/env node

'use strict';

var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');
rosnodejs.initNode('/listener_node', { onTheFly: true }).then(function (rosNode) {
  rosNode.subscribe('/processed_image1', 'sensor_msgs/Image', function (data) {
    console.log('processed_image', data);
  }, { queueSize: 1,
    throttleMs: 10 });
});