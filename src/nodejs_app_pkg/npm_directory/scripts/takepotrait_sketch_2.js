const electron = require('electron')
const remote = electron.remote
const mainProcess = remote.require('./main')

var image_path = "/home/chibike/Projects/ArtBot-Project/Archive/artbot_ws/src/image_processing_pkg/images/box.png";
var canvas_name = "my_canvas_div";
var canvas = document.getElementById(canvas_name);
var ctx = canvas.getContext('2d');

var ros_bridge = new mainProcess.ros_bridge();

function update_img()
{
	var data = ros_bridge.get_image();
	console.log("Hello");
}


setInterval(update_img, 1000);