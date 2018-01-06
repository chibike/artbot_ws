const electron = require('electron')
const remote = electron.remote
const mainProcess = remote.require('./main')

var img;
var ros_bridge;

function setup()
{
	var my_canvas = createCanvas(windowWidth, windowHeight);
	my_canvas.parent('my_canvas_div');
	
  img = loadImage("../images/marguerite-daisy-beautiful-beauty.jpg");
  background(img);

  ros_bridge = new mainProcess.ros_bridge();

  frameRate(5);
}

function draw()
{
  //update_background();
  background(img);
}

function windowResized()
{
	resizeCanvas(windowWidth, windowHeight);
}

function update_background()
{
  var data = ros_bridge.get_image();
  if (data)
  {
    var image = data.image;
    var width = data.width;
    var height = data.height;

    var new_img = createImage(width, height);
    new_img.loadPixels();

    for (var x=0; x<width; x++)
    {
      for (var y=0; y<height; y++)
      {
        var index = x*y*3;
        var g = image[index];
        var b = image[index + 1];
        var r = image[index + 2];

        var color = [r,g,b, 255];
        new_img.set(x, y, color);
      }
    }

    new_img.updatePixels();
    img = new_img;
  }
}