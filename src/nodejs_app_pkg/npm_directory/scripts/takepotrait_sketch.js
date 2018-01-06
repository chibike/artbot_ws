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
  ros_bridge.add_to_image_update_callback_list(update_background);

  frameRate(5);
  //noLoop();
}

function draw()
{
  background(img);
}

function windowResized()
{
	resizeCanvas(windowWidth, windowHeight);
}

function update_background(img)
{
  if (img)
  {
    //background(img);
    //console.log("updating background...", img);
  }
}