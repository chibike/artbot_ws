#!/usr/bin/env node

'use strict';

var rosnodejs = require('../../node_modules/rosnodejs/dist/index.js');
rosnodejs.initNode('/listener_node', { onTheFly: true }).then(function (rosNode) {
  
  console.log("Simple Script")

});