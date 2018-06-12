#pragma once

#include <math.h>
#include "pi_point_header.h"

namespace blink
{

double measure(Point point1, Point point2)
{
	double distance = sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
	return distance;
}

struct RectInfo
{
	Point bottom_right;
	Point center;
	Point top_left;
	double height;
	double width;
	double radius;
	double area;
	double perimeter;
};

}

#include "pi_line_header.h"
#include "pi_path_header.h"
#include "pi_pathfinder_header.h"