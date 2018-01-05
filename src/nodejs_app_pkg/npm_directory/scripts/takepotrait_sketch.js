var img;

 
var numBalls = 12;
var gravity = 0.9;
var balls = new Array();

function setup()
{
	var my_canvas = createCanvas(windowWidth, windowHeight);
	my_canvas.parent('my_canvas_div');
	img = loadImage("../images/marguerite-daisy-beautiful-beauty.jpg");

	frameRate(5);
	colorMode(HSB, 100);

	for (var i = 0; i < numBalls; i++)
	{
    	balls[i] = new Ball(random(width), random(height), random(30, 70), i, balls);
    }
}

function draw()
{
	background(img);
	for (var i=0; i<balls.length; i++)
	{
    	balls[i].collide();
    	balls[i].move();
    	balls[i].display();
    }
}

function windowResized()
{
	resizeCanvas(windowWidth, windowHeight);
}

var Ball = class
{
  constructor(xin, yin, din, idin, oin)
  {
    this.x = xin;
    this.y = yin;
    this.diameter = din;
    this.id = idin;
    this.others = oin;
    this.vx = 0;
    this.vy = 0;
    this.spring = random(50)/100.0;
    this.friction = -1.0*random(50, 100)/100.0;

    this.stroke_h = int(random(100));
	this.fill_h = int(random(100));
  } 
  
  collide()
  {
    for (var i = this.id + 1; i < this.others.length; i++)
    {
      var dx = this.others[i].x - this.x;
      var dy = this.others[i].y - this.y;
      var distance = sqrt(dx*dx + dy*dy);
      var minDist = this.others[i].diameter/2 + this.diameter/2;
      if (distance < minDist)
      { 
        var angle = atan2(dy, dx);
        var targetX = this.x + cos(angle) * minDist;
        var targetY = this.y + sin(angle) * minDist;
        var foo = (targetX - this.others[i].x);
        var bar = (targetY - this.others[i].y);
        this.vx -= foo * this.spring;
        this.vy -= bar * this.spring;
        this.others[i].vx += foo * this.others[i].spring;
        this.others[i].vy += bar * this.others[i].spring;
      }
    }   
  }
  
  move()
  {
    this.vy += gravity;
    this.x += this.vx;
    this.y += this.vy;
    if (this.x + this.diameter/2 > width)
    {
      this.x = width - this.diameter/2;
      this.vx *= this.friction; 
    }
    else if (this.x - this.diameter/2 < 0)
    {
      this.x = this.diameter/2;
      this.vx *= this.friction;
    }
    if (this.y + this.diameter/2 > height)
    {
      this.y = height - this.diameter/2;
      this.vy *= this.friction; 
    } 
    else if (this.y - this.diameter/2 < 0)
    {
      this.y = this.diameter/2;
      this.vy *= this.friction;
    }
  }
  
  display()
  {
  	stroke(this.stroke_h, 100, 100);
	fill(this.fill_h, 100, 100, 10);
    ellipse(this.x, this.y, this.diameter, this.diameter);
  }
}