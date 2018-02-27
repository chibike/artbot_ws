const electron = require('electron')
const remote = electron.remote
const mainProcess = remote.require('./main')
let serial_port = mainProcess.serial_port;

var img;
var new_img;
// var ros_bridge;

function setup()
{
	var my_canvas = createCanvas(windowWidth, windowHeight);
	my_canvas.parent('my_canvas_div');
	
  img = loadImage("../images/marguerite-daisy-beautiful-beauty.jpg");

  new_img = createImage(640/2, 480/2);
  new_img.loadPixels();

  // ros_bridge = new mainProcess.ros_bridge();

  frameRate(5);
}

function draw()
{
  update_background();
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
    console.log("Yeah");
    //var image = data.image;
    //new_img.pixels = data;
    //new_img.updatePixels();
    //img = new_img;
  }
}