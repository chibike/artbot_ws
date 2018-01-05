#!/usr/bin/env node

'use strict';

var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');
rosnodejs.initNode('/image_viewer_node', { onTheFly: true }).then(function (rosNode) {
  rosNode.subscribe('/image', 'sensor_msgs/Image', function (data) {
    console.log('image', data);
  }, { queueSize: 1,
    throttleMs: 10 });
});