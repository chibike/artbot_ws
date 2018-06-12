#pragma once

#include "pi_line_header.h"

namespace blink
{

Line::Line(Point point1, Point point2)
{
	start = point1;
	end = point2;

	compute();
}

void Line::compute()
{
	line.push_back(start);
	line.push_back(end);

	m = get_gradient();
	ref_point = start;
	length = get_length();
	angle = get_angle();
}

Point Line::_rotate_coord(Point coord, double angle, Point about_point)
{
	double c1 = about_point.x;
	double c2 = about_point.y;

	double xold = coord.x;
	double yold = coord.y;

	Eigen::MatrixXd x(2,1);
	Eigen::MatrixXd y(2,2);
	Eigen::MatrixXd z(2,1);

	x(0,0) = c1; x(1,0) = c2;
	y(0,0) = cos(angle); y(0, 1) = -1.0 * sin(angle);
	y(1,0) = sin(angle); y(1, 1) = cos(angle);
	z(0,0) = xold - c1; z(1,0) = yold - c2;

	Eigen::MatrixXd a = x + (y*z);

	Point new_point = Point(a(0,0), a(1,0));
	return new_point;
}

bool Line::rotate(double angle)
{//rotate start point about midpoint

	Point about_point = get_midpoint();
	return rotate(angle, about_point);
}

bool Line::rotate(double angle, Point about_point)
{

	start = _rotate_coord(start, angle, about_point);
	end = _rotate_coord(end, angle, about_point);

	compute();

	return true;
}

bool Line::translate(double x, double y)
{
	Point translation_vector = Point(x, y);
	
	start = start + translation_vector;
	end = end + translation_vector;

	compute();
	return true;
}

bool Line::shear(double x, double y)
{
	return true;
}

bool Line::scale(double mag_x, double mag_y)
{
	Point scale_vector = Point(mag_x, mag_y);
	
	start = start * scale_vector;
	end = end * scale_vector;

	compute();
	return true;
}

bool Line::scale(double mag_x, double mag_y, Point about_point)
{
	double dx = mag_x * length * 0.5 * std::cos(angle);
	double dy = mag_y * length * 0.5 * std::sin(angle);

	// swap .y-dy for .y+dy when working in cartesian coord
	start = Point(about_point.x-dx, about_point.y-dy);
	end   = Point(about_point.x+dx, about_point.y+dy);
	compute();

	return true;
}

bool Line::scale(double mag)
{
	Point about_point = get_midpoint();
	double new_length = mag * length * 0.5;

	double dx = new_length * std::cos(angle);
	double dy = new_length * std::sin(angle);

	// swap .y-dy for .y+dy when working in cartesian coord
	start = Point(about_point.x-dx, about_point.y-dy);
	end   = Point(about_point.x+dx, about_point.y+dy);
	compute();

	return true;
}

bool Line::scale(double mag, Point about_point)
{
	double new_length = mag * length * 0.5;

	double dx = new_length * std::cos(angle);
	double dy = new_length * std::sin(angle);

	// swap .y-dy for .y+dy when working in cartesian coord
	start = Point(about_point.x-dx, about_point.y-dy);
	end   = Point(about_point.x+dx, about_point.y+dy);
	compute();

	return true;
}

// void Line::translate(double dx, double dy)
// {
// 	Point point = Point(dx, dy);
// 	start.add(point);
// 	end.add(point);
// 	compute();
// }

double Line::get_length()
{
	return measure(start, end);
}

double Line::get_gradient()
{
	return tan(get_angle());
}

double Line::get_angle()
{
	double dx = end.x - start.x;
	double dy = end.y - start.y;
	double angle = atan2(dy, dx);
	return angle;
}

Point Line::get_midpoint()
{
	double x = start.x + ((end.x - start.x) * 0.5);
	double y = start.y + ((end.y - start.y) * 0.5);
	Point midpoint = Point(x, y);
	return midpoint;
}

Point Line::get_point(double position)
{
	double x = start.x + ((end.x - start.x) * position);
	double y = start.y + ((end.y - start.y) * position);
	return Point(x,y);
}

void Line::plot(std::string title)
{
	//pass
}

void Line::plot()
{
	plot("Line-01");
}

bool Line::is_on_line(Point point)
{
	// This works based on the principle
	// A--C-----B; where AC,CB,AB are a lines
	// and AC + CB == AB

	double ac = measure(start, point);
	double cb = measure(point, end);
	double ab = measure(start, end);

	double diff = (ac + cb) - ab;
	return abs(diff) <= 0.001;
}

Point Line::get_intersecting_point(Line other)
{
	// line 1
	double x1 = start.x; double x2 = end.x;
	double y1 = start.y; double y2 = end.y;

	// line 2
	double x3 = other.start.x; double x4 = other.end.x;
	double y3 = other.start.y; double y4 = other.end.y;

	Eigen::MatrixXd a(2,2);
	Eigen::MatrixXd b(2,2);
	Eigen::MatrixXd c(2,2);
	Eigen::MatrixXd d(2,2);
	Eigen::MatrixXd e(2,2);
	Eigen::MatrixXd f(2,2);

	Eigen::MatrixXd A(2,2);
	Eigen::MatrixXd B(2,2);
	Eigen::MatrixXd C(2,2);

	a(0,0) = x1;a(0, 1) = y1; a(1,0) = x2;a(1, 1) = y2;

	b(0,0) = x1;b(0, 1) = 1;  b(1,0) = x2;b(1, 1) = 1;

	c(0,0) = x3;c(0, 1) = y3; c(1,0) = x4;c(1, 1) = y4;

	d(0,0) = x3;d(0, 1) = 1;  d(1,0) = x4;d(1, 1) = 1;

	e(0,0) = y1;e(0, 1) = 1;  e(1,0) = y2;e(1, 1) = 1;

	f(0,0) = y3;f(0, 1) = 1;  f(1,0) = y4;f(1, 1) = 1;

	A(0,0) = a.determinant(); A(0,1) = b.determinant();
	A(1,0) = c.determinant(); A(1,1) = d.determinant();

	B(0,0) = b.determinant(); B(0,1) = e.determinant();
	B(1,0) = d.determinant(); B(1,1) = f.determinant();

	C(0,0) = a.determinant(); C(0,1) = e.determinant();
	C(1,0) = c.determinant(); C(1,1) = f.determinant();

	double x = A.determinant() / B.determinant();
	double y = C.determinant() / B.determinant();

	Point point = Point(x, y);
	if (!(is_on_line(point) && other.is_on_line(point)))
	{
		Point new_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
		return new_point;
	}

	return point;
}

void Line::draw(cv::Mat image, cv::Scalar color)
{
	draw(image, color, 1, 8, 0);
}

void Line::draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift)
{
	cv::Point start_point = cv::Point(start.x, start.y);
	cv::Point end_point   = cv::Point(end.x, end.y);

	cv::line(image, start_point, end_point, color, stroke_width, line_type, shift);
}

std::string Line::get_as_string()
{
	// '[[0, 4], [1, 5]]'
	std::stringstream ss;
	ss << "[";
	ss << "[" << start.x << "," << start.y << "],";
	ss << "[" << end.x   << "," << end.y   << "]" ;
	ss << "]";
	return ss.str();
}

std::ostream& operator<<(std::ostream& os, const Line& line)
{
	os << "Line(" << line.start << ", " << line.end << ")";
	return os;
}

}