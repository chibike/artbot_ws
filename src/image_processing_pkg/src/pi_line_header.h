#pragma once

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include "Eigen/Dense"
#include "pi_arithmetic.h"
#include "opencv2/opencv.hpp"
#include "pi_object_header_file.h"

namespace blink
{

class Line
{
public:
	Point          start;
	Point          end;
	std::vector<Point>  line;
	double         m;
	Point          ref_point;
	double         length;
	double         angle;

	Line(Point point1, Point point2);

	void   compute();
	Point  _rotate_coord(Point coord, double angle, Point about_point);
	double get_length();
	double get_gradient();
	double get_angle();
	Point  get_midpoint();
	Point  get_point(double position); // returns a point at that position on the line; ranges from 0..1
	void   plot();
	void   plot(std::string title);
	bool   is_on_line(Point point);
	Point  get_intersecting_point(Line other);
	void   draw(cv::Mat image, cv::Scalar color);
	void   draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift);

	bool translate(double x, double y);
	bool shear(double x, double y);
	bool scale(double mag);
	bool scale(double mag, Point about_point);
	bool scale(double mag_x, double mag_y);
	bool scale(double mag_x, double mag_y, Point about_point);
	bool rotate(double angle);
	bool rotate(double angle, Point about_point);
	// void translate(double dx, double dy);

	friend std::ostream& operator<<(std::ostream& os, const Line& line);

	std::string get_as_string();
};

}