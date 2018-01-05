#!/usr/bin/env node

'use strict';

var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');
rosnodejs.initNode('/listener_node', { onTheFly: true }).then(function (rosNode) {
  rosNode.subscribe('/chatter', 'std_msgs/String', function (data) {
    console.log('string', data);
  }, { queueSize: 1,
    throttleMs: 10 });
});