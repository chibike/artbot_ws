const electron = require('electron')
const remote = electron.remote
const mainProcess = remote.require('./main')

var ros_bridge = new mainProcess.ros_bridge();

var canvas_name = "canvas";
var canvas = document.getElementById(canvas_name);
var ctx = canvas.getContext('2d');

var image_width = 640/2;
var image_height = 480/2;
var image_size = image_width * image_height
var buffer = new Uint8ClampedArray(image_width * image_height * 4);

function update_img()
{
	var data = ros_bridge.get_image();
	if (data)
	{
		var image = data.image;
		for (var i=0; i<image_size; i++)
		{
			buffer[i]   = image[i];
			buffer[i+1] = image[i+1];
			buffer[i+2] = image[i+2];

			// Opacity
			buffer[i+3] = 255;
		}

		var im_data = ctx.createImageData(image_width, image_height);
		im_data.data.set(buffer);
		ctx.putImageData(im_data, 0, 0);
		//ctx.drawImage(im_data, 0, 0, image_width, image_height, 0, 0, canvas.width, canvas.height);
	}
	else
	{
		console.log(data);
	}
}

var fps = 5;
var update_interval = (1/fps)*1000;
setInterval(update_img, update_interval);