#pragma once

#include <math.h>
#include <vector>
#include <ctype.h>
#include <iostream>

#include "pi_objects.h"
#include "opencv2/opencv.hpp"

namespace blink
{

class Pathifier
{
// private:
// 	Path _path;

public:
	Pathifier();

	bool from_circle (double x, double y, double r, unsigned int steps, Path &path_out);
	bool from_ellipse(double x, double y, double rx, double ry, unsigned int steps, Path &path_out);
	bool from_rectangle(double x, double y, double width, double height, Path &path_out);
	bool from_polygons(std::vector<double> xs, std::vector<double> ys, bool closed, Path &path_out);
	bool from_arc(double x, double y, double rx, double ry, double x_axis_rotate, double large_arc_flag, double sweep_flag, Path &path_out);
	bool from_cubic_curve(double x, double y, double x1, double y1, double x2, double y2, Path &path_out);
	bool from_quadratic_curve(double x, double y, double x1, double y1, Path &path_out);
};




Pathifier::Pathifier()
{
	// pass
}

bool Pathifier::from_circle(double x, double y, double r, unsigned int steps, Path &path_out)
{
	Point center = Point(x,y);
	path_out = Path(center, r, 2.0*M_PI, steps);
	return true;
}

bool Pathifier::from_ellipse(double x, double y, double rx, double ry, unsigned int steps, Path &path_out)
{
	Point center = Point(x,y);
	path_out = Path(center, rx, ry, 2.0*M_PI, steps);
	return true;
}

bool Pathifier::from_rectangle(double x, double y, double width, double height, Path &path_out)
{
	cv::Rect rect = cv::Rect(x, y, width, height);
	path_out = Path(rect);
	return true;
}

bool Pathifier::from_polygons(std::vector<double> xs, std::vector<double> ys, bool closed, Path &path_out)
{
	std::vector<Point> points;
	points.reserve(xs.size());

	for (int i=0; i<xs.size(); i++)
	{
		points.emplace_back(Point(xs.at(i), ys.at(i)));
	}

	path_out = Path(points, closed);
	return true;
}

}