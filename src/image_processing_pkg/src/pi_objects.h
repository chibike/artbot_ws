#ifndef BLINK_PI_OBJECTS_H
#define BLINK_PI_OBJECTS_H

#include <vector>
#include <math.h>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <sstream>

namespace blink
{

using namespace std;
using Eigen::MatrixXd;

double maximum_element(std::vector<double> list)
{
	double max_element = list.at(0);
	for (int i=0; i<list.size(); i++)
	{
		double element = list.at(i);

		if (element > max_element)
		{
			max_element = element;
		}
	}
	return max_element;
}

double minimum_element(std::vector<double> list)
{
	double min_element = list.at(0);
	for (int i=0; i<list.size(); i++)
	{
		double element = list.at(i);

		if (element < min_element)
		{
			min_element = element;
		}
	}
	return min_element;
}

class Point
{
public:
	double          x;
	double          y;
	double          r;
	double          theta;
	vector<double>  point;
	vector<double>  point_polar;

	Point();
	Point(double xx, double yy);

	bool            isinf();
	Point           add(Point other);
	Point           sub(Point other);
	Point           mul(Point other);
	Point           div(Point other);
	bool            equal(Point other);
	vector<double>  get_as_polar();

	void calculate();
	void set_zero();
	void set_inf();
};

class Line
{
public:
	Point          start;
	Point          end;
	vector<Point>  line;
	double         m;
	Point          ref_point;
	double         length;
	double         angle;

	Line(Point point1, Point point2);

	void   compute();
	Point  _rotate_coord(Point coord, double angle, Point about_point);
	void   rotate(double angle);
	void   rotate(double angle, Point about_point);
	double get_length();
	double get_gradient();
	double get_angle();
	Point  get_midpoint();
	void   plot();
	void   plot(string title);
	bool   is_on_line(Point point);
	void   scale(double mag);
	void   translate(double dx, double dy);
	Point  get_intersecting_point(Line other);
	void   draw(cv::Mat image, cv::Scalar color);
	void   draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift);
};

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

class Path
{
public:
	std::vector<Point> data_points;
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<Line> boundary_lines;

	bool closed;
	RectInfo rect_info;

	Path(std::vector<Point> raw_point_data, bool closed);
	Path(std::vector<cv::Point> raw_point_data, bool closed);

	Point get_start_point();
	Point get_end_point();
	RectInfo get_rect_info();
	std::vector<Line> get_boundary_lines();
	std::vector<Point> get_intersecting_points(Line line);
	std::vector<Line> get_shading_lines(double spacing, double angle, double padding);
	std::vector<Line> get_intersecting_lines(double spacing, double angle);
	std::vector<Point> get_intersecting_points(Path other);
	bool is_touching(Point point_1);
	bool is_touching(Line line);
	bool is_within(Point point_1);
	bool is_within(Line line);
	void rotate(double angle);
	void rotate(double angle, Point pivot);
	void draw(cv::Mat image, cv::Scalar color);
	void draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift);

	// void filter_points(double min_distance);
	// void filter_points(double min_distance, double max_distance);

	std::string get_as_string();

	void calculate();
	static bool _filter_x_function(Point current, Point next)
	{
		return current.x < next.x;
	}
};

double measure(Point point1, Point point2)
{
	double distance = sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
	return distance;
}

class Pathfinder
{
public:
	Pathfinder();
	
	Path unite(Path path_1, Path path_2);
	Path subtract(Path path_1, Path path_2);

	std::vector<Line> shade_frame(double top_x, double top_y, double frame_width, double frame_height, std::vector<Path> exclude_regions);
	std::vector<Line> shade_frame(double top_x, double top_y, double frame_width, double frame_height, double spacing, double angle, std::vector<Path> exclude_regions);

	static bool _filter_x_function(Point current, Point next)
	{
		return current.x < next.x;
	}
};



std::vector<Line> Pathfinder::shade_frame(double top_x, double top_y, double frame_width, double frame_height, std::vector<Path> exclude_regions)
{
	return shade_frame(top_x, top_y, frame_width, frame_height, 10, 0., exclude_regions);
}

std::vector<Line> Pathfinder::shade_frame(double top_x, double top_y, double frame_width, double frame_height, double spacing, double angle, std::vector<Path> exclude_regions)
{
	std::vector<Point> points;
	points.push_back(Point(top_x, top_y));
	points.push_back(Point(top_x + frame_width, top_y));
	points.push_back(Point(top_x + frame_width, top_y + frame_height));
	points.push_back(Point(top_x, top_y + frame_height));

	Path frame_path = Path(points, false);
	std::vector<Line> intersecting_lines = frame_path.get_intersecting_lines(spacing, angle);
	std::vector<Line> exclude_regions_boundary_lines;

	// std::vector<Line> intersecting_lines;
	// double _h = 90;
	// Point p1 = Point(top_x,_h);
	// Point p2 = Point(top_x + frame_width, _h);
	// intersecting_lines.push_back(Line(p1, p2));

	for (int i=0; i<exclude_regions.size(); i++)
	{
		Path region = exclude_regions.at(i);
		exclude_regions_boundary_lines.insert(exclude_regions_boundary_lines.end(), region.boundary_lines.begin(), region.boundary_lines.end());
	}

	std::vector<Line> shading_lines;
	for (int i=0; i<intersecting_lines.size(); i++)
	{
		Line intersecting_line = intersecting_lines.at(i);
		
		std::vector<Point> intersection_points;
		for (int j=0; j<exclude_regions_boundary_lines.size(); j++)
		{
			Line line = exclude_regions_boundary_lines.at(j);
			Point intersection_point = intersecting_line.get_intersecting_point(line);
			if (!intersection_point.isinf())
			{
				intersection_points.push_back(intersection_point);
			}
		}

		std::sort(intersection_points.begin(), intersection_points.end(), _filter_x_function);

		intersection_points.insert(intersection_points.begin(), intersecting_line.start);
		intersection_points.insert(intersection_points.end(), intersecting_line.end);

		Point p1 = intersection_points.at(0);
		for (int j=1; j<intersection_points.size(); j++)
		{
			if (intersection_points.at(j).x - p1.x < 0.001)
			{
				continue;
			}

			Line line = Line(p1, intersection_points.at(j));
			p1 = intersection_points.at(j);

			shading_lines.push_back(line);
		}
	}

	// filter shading lines
	std::vector<Line> shading_lines_buffer;
	for (int i=0; i<shading_lines.size(); i++)
	{
		Line shading_line = shading_lines.at(i);
		bool is_within_exclude_region = false;

		for (int j=0; j<exclude_regions.size(); j++)
		{
			Path region = exclude_regions.at(j);
			is_within_exclude_region = region.is_within(shading_line);

			if (is_within_exclude_region)
			{
				break;
			}
		}

		cout << "------------------" << endl << endl << endl;

		if (!is_within_exclude_region)
		{
			shading_lines_buffer.push_back(shading_line);
		}
	}

	return shading_lines_buffer;
}

Point::Point()
{
	x = 0;
	y = 0;
	
	calculate();
}

Point::Point(double xx, double yy)
{
	x = xx;
	y = yy;

	calculate();
}

void Point::set_zero()
{
	x = 0;
	y = 0;
	
	calculate();
}

void Point::set_inf()
{
	x = std::numeric_limits<double>::infinity();
	y = std::numeric_limits<double>::infinity();
	
	calculate();
}

void Point::calculate()
{
	r = sqrt(pow(x, 2) + pow(y, 2));
	theta = atan2(y,x);

	point.push_back(x);
	point.push_back(y);
	point_polar.push_back(r);
	point_polar.push_back(theta);
}

Point Point::add(Point other)
{
	Point new_point(x + other.x, y + other.y);
	return new_point;
}

Point Point::sub(Point other)
{
	Point new_point(x - other.x, y - other.y);
	return new_point;
}

Point Point::mul(Point other)
{
	Point new_point(x * other.x, y * other.y);
	return new_point;
}

Point Point::div(Point other)
{
	Point new_point(x / other.x, y / other.y);
	return new_point;
}

bool Point::equal(Point other)
{
	return (std::abs(x - other.x) < 0.001 && std::abs(y - other.y) < 0.001);
}

vector<double> Point::get_as_polar()
{
	return point_polar;
}

bool Point::isinf()
{
	return std::isinf(x) || std::isinf(y);
}

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

	MatrixXd x(2,1);
	MatrixXd y(2,2);
	MatrixXd z(2,1);

	x(0,0) = c1; x(1,0) = c2;
	y(0,0) = cos(angle); y(0, 1) = -1.0 * sin(angle);
	y(1,0) = sin(angle); y(1, 1) = cos(angle);
	z(0,0) = xold - c1; z(1,0) = yold - c2;

	MatrixXd a = x + (y*z);

	Point new_point = Point(a(0,0), a(1,0));
	return new_point;
}

void Line::rotate(double angle)
{//rotate start point about midpoint

	Point about_point = get_midpoint();

	start = _rotate_coord(start, angle, about_point);
	end = _rotate_coord(end, angle, about_point);

	compute();
}

void Line::rotate(double angle, Point about_point)
{//rotate start point about midpoint

	start = _rotate_coord(start, angle, about_point);
	end = _rotate_coord(end, angle, about_point);

	compute();
}

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
	double x = (start.x + end.x)/2.0;
	double y = (start.y + end.y)/2.0;
	Point midpoint = Point(x, y);
	return midpoint;
}

void Line::plot(string title)
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

void Line::scale(double mag)
{
	Point midpoint = get_midpoint();
	double new_length = mag * length * 0.5;

	double dx = new_length * std::cos(angle);
	double dy = new_length * std::sin(angle);

	// swap .y-dy for .y+dy when working in cartesian coord
	start = Point(midpoint.x-dx, midpoint.y-dy);
	end   = Point(midpoint.x+dx, midpoint.y+dy);
	compute();
}

void Line::translate(double dx, double dy)
{
	Point point = Point(dx, dy);
	start.add(point);
	end.add(point);
	compute();
}

Point Line::get_intersecting_point(Line other)
{
	// line 1
	double x1 = start.x; double x2 = end.x;
	double y1 = start.y; double y2 = end.y;

	// line 2
	double x3 = other.start.x; double x4 = other.end.x;
	double y3 = other.start.y; double y4 = other.end.y;

	MatrixXd a(2,2);
	MatrixXd b(2,2);
	MatrixXd c(2,2);
	MatrixXd d(2,2);
	MatrixXd e(2,2);
	MatrixXd f(2,2);

	MatrixXd A(2,2);
	MatrixXd B(2,2);
	MatrixXd C(2,2);

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


Path::Path(std::vector<Point> raw_point_data, bool is_closed)
{
	// verify arrangement
	// if (raw_point_data.size() > 1)
	// {
	// 	Point p1 = raw_point_data.at(0);
	// 	Point p2 = raw_point_data.at(1);

	// 	if (p2.x < p1.x)
	// 	{
	// 		std::reverse(raw_point_data.begin(), raw_point_data.end());
	// 	}
	// }

	data_points = raw_point_data;
	closed = is_closed;

	calculate();
}

Path::Path(std::vector<cv::Point> raw_point_data, bool is_closed)
{
	std::vector<Point> points;

	for (int i=0; i<raw_point_data.size(); i++)
	{
		cv::Point cv_point = raw_point_data.at(i);
		Point point = Point(cv_point.x, cv_point.y);

		points.push_back(point);
	}

	data_points = points;
	closed = is_closed;

	calculate();
}

Point Path::get_start_point()
{
	return data_points.at(0);
}

Point Path::get_end_point()
{
	return data_points.at(data_points.size()-1);
}

RectInfo Path::get_rect_info()
{
	return rect_info;
}

std::vector<Line> Path::get_boundary_lines()
{
	return boundary_lines;
}

std::vector<Point> Path::get_intersecting_points(Line line)
{
	std::vector<Point> intersecting_points;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line current_line = boundary_lines.at(i);
		Point intersecting_point = line.get_intersecting_point(current_line);

		if (!intersecting_point.isinf())
		{
			intersecting_points.push_back(intersecting_point);
		}
	}

	return intersecting_points;
}

std::vector<Point> Path::get_intersecting_points(Path other)
{
	std::vector<Point> intersecting_points;
	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		for (int j=0; j<other.boundary_lines.size(); j++)
		{
			Line other_line = other.boundary_lines.at(j);
			Point intersecting_point = line.get_intersecting_point(other_line);

			if (!intersecting_point.isinf())
			{
				intersecting_points.push_back(intersecting_point);
			}
		}
	}
	return intersecting_points;
}

std::vector<Line> Path::get_shading_lines(double spacing, double angle, double padding)
{
	std::vector<Line> lines;

	double min_x = rect_info.top_left.x - padding;
	double min_y = rect_info.top_left.y - padding;

	double max_x = rect_info.bottom_right.x + padding;
	double max_y = rect_info.bottom_right.y + padding;

	double cx = (max_x - min_x)/2.0 + min_x;
	double cy = (max_y - min_y)/2.0 + min_y;

	Point about_point = Point(cx, cy);

	// create lines
	for (double y=min_y; y<=max_y; y+=spacing)
	{
		Point start = Point(min_x, y);
		Point end   = Point(max_x, y);
		Line line   = Line(start, end);

		line.rotate(angle, about_point);
		lines.push_back(line);
	}

	return lines;
}

std::vector<Line> Path::get_intersecting_lines(double spacing, double angle)
{
	double padding = std::max(rect_info.width, rect_info.height);
	std::vector<Line> lines = get_shading_lines(spacing, angle, padding);


	std::vector<Line> intersecting_lines;
	for (int i=0; i<lines.size(); i++)
	{
		Line line = lines.at(i);
		std::vector<Point> intersecting_points = get_intersecting_points(line);

		std::sort(intersecting_points.begin(), intersecting_points.end(), _filter_x_function);

		for (int j=0; j<std::max((int)intersecting_points.size()-1, 0); j+=2)
		{
			Point start = intersecting_points.at(j);
			Point end   = intersecting_points.at(j+1);

			Line line = Line(start, end);
			intersecting_lines.push_back(line);
		}
	}

	return intersecting_lines;
}

void Path::calculate()
{
	if (!closed)
	{
		Point point = data_points.at(0);
		data_points.push_back(point);
	}

	for (int i=0; i<data_points.size(); i++)
	{
		Point point = data_points.at(i);
		xs.push_back(point.x);
		ys.push_back(point.y);
	}

	boundary_lines.clear();
	int perimeter = 0;
	for (int i=0; i<data_points.size()-1; i++)
	{
		Point start = data_points.at(i);
		Point end   = data_points.at(i+1);

		Line line = Line(start, end);
		boundary_lines.push_back(line);

		perimeter += std::abs(line.length);
	}

	rect_info.top_left     = Point(minimum_element(xs), minimum_element(ys));
	rect_info.width        = maximum_element(xs) - rect_info.top_left.x;
	rect_info.height       = maximum_element(ys) - rect_info.top_left.y;
	rect_info.bottom_right = Point(rect_info.top_left.x + rect_info.width, rect_info.top_left.y + rect_info.height);
	rect_info.area         = rect_info.width * rect_info.height;
	rect_info.radius       = std::sqrt(std::pow(rect_info.width, 2) + std::pow(rect_info.height, 2)) / 2.0;
	rect_info.center       = Point(rect_info.top_left.x + (rect_info.width/2.0), rect_info.top_left.y + (rect_info.height/2.0));
	rect_info.perimeter    = perimeter;
}

void Path::rotate(double angle)
{
	Point pivot = rect_info.center;
	rotate(angle, pivot);
}

void Path::rotate(double angle, Point pivot)
{
	data_points.clear();
	closed = false;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.rotate(angle, pivot);
		data_points.push_back(line.start);
	}

	calculate();
}

bool Path::is_within(Point point_1)
{ // uses raying casting algorithm to solve point in polygon
	
	Point point_2 = Point(point_1.x + rect_info.top_left.x + rect_info.width + 10, point_1.y);
	Line ray_line = Line(point_1, point_2);

	std::vector<Point> intersecting_points = get_intersecting_points(ray_line);

	if (intersecting_points.size() % 2 == 0)
	{ // is even
		return false;
	}
	else if (intersecting_points.size() == 1)
	{
		if (intersecting_points.at(0).equal(point_1))
		{ // justing touching
			return false;
		}
	}

	return true;
}

bool Path::is_within(Line line)
{
	std::vector<Point> points = get_intersecting_points(line);

	cout << "--- line is within ---" << endl;
	cout << "y = " << (double)line.start.y << endl;
	cout << "start, end = " << (double)line.start.x << " - " << (double)line.end.x << endl;
	cout << "length = " << (double)line.get_length() << endl;
	cout << "--- intersection points ---" << endl;
	cout << "\t";
	cout << "points = ";
	for (int i=0; i<points.size(); i++)
	{
		Point p = points.at(i);
		cout << "px_" << ((double) p.x) << ", ";
	}
	cout << endl;


	if (points.size() == 1)
	{
		if (is_touching(line.start) && !(line.start.equal(points.at(0))))
		{
			cout << "01 is_within: " << "true";
			cout << " d" << points.at(0).x - line.start.x << " d";
			cout << endl;
			return true;
		}
		else if (is_touching(line.end) && !(line.end.equal(points.at(0))))
		{
			cout << "02 is_within: " << "true";
			cout << endl;
			return true;
		}
	}
	else if (points.size() == 2)
	{
		std::vector<Point> other_points;
		other_points.push_back(line.start);
		other_points.push_back(line.end);

		std::sort(points.begin(), points.end(), _filter_x_function);
		std::sort(other_points.begin(), other_points.end(), _filter_x_function);

		double dx_0 = other_points.at(0).x - points.at(0).x;
		double dx_1 = points.at(1).x - other_points.at(1).x;

		cout << " dx_" << dx_0 << "__" << dx_1 << "_";

		return (dx_0 >= -0.0001 && dx_1 >= -0.0001);
	}

	if (is_within(line.start)  || is_within(line.end))
	{
		cout << "03a is_within: " << "true";
		cout << endl;
		return true;
	}
	else if ( is_touching(line.start) && is_touching(line.end) )
	{
		cout << "03c is_within: " << "true";
		cout << endl;
		return true;
	}
	else
	{
		cout << "04 is_within: " << "false";
		cout << endl;
		return false;
	}
}

bool Path::is_touching(Point point_1)
{ // uses raying casting algorithm to solve point in polygon
	
	Point point_2 = Point(point_1.x + rect_info.top_left.x + rect_info.width + 10, point_1.y);
	Line ray_line = Line(point_1, point_2);

	std::vector<Point> intersecting_points = get_intersecting_points(ray_line);

	return !(intersecting_points.size() % 2 == 0);
}

bool Path::is_touching(Line line)
{
	std::vector<Point> points = get_intersecting_points(line);
	return is_touching(line.start) || is_touching(line.end) || (points.size() > 0);
}

void Path::draw(cv::Mat image, cv::Scalar color)
{
	draw(image, color, 1, 8, 0);
}

void Path::draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift)
{
	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.draw(image, color, stroke_width, line_type, shift);
	}
}

std::string Path::get_as_string()
{
	// '[[0, 4], [1, 5], [2, 6], [3, 7]]'
	std::stringstream ss;
	ss << "[";

	int last_index=data_points.size()-1;
	for (int i=0; i<last_index; i++)
	{
		ss << "[" << xs.at(i) << "," << ys.at(i) << "],";
	}

	if (last_index >= 0)
	{
		ss << "[" << xs.at(last_index) << "," << ys.at(last_index) << "]]";
	}
	else
	{
		ss << "]";
	}

	

	/*const char *cstr = path.get_as_string().c_str();*/
	return ss.str();
}

Pathfinder::Pathfinder()
{
}

Path Pathfinder::unite(Path path_1, Path path_2)
{
	std::vector<Line> boundary_lines_1 = path_1.get_boundary_lines();
	std::vector<Line> boundary_lines_2 = path_2.get_boundary_lines();

	std::vector<Point> outline;
	for (int i=0; i<boundary_lines_1.size(); i++)
	{// trace boundarys

		Line current_line = boundary_lines_1.at(i);
		outline.push_back(current_line.start);

		Point intersecting_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
		double distance_of_point = measure(current_line.start, intersecting_point);
		int index = 0;

		for (int j=0; j<boundary_lines_2.size(); j++)
		{// get closest intersection

			Line line = boundary_lines_2.at(j);
			Point point = current_line.get_intersecting_point(line);
			double distance = measure(current_line.start, point);
			if (distance < distance_of_point)
			{
				distance_of_point = distance;
				intersecting_point = point;
				index = j;
			}
		}

		if (!intersecting_point.isinf())
		{
			outline.push_back(intersecting_point);
			for (int k=(index+1)%boundary_lines_2.size(); k<boundary_lines_2.size(); k++)
			{
				current_line = boundary_lines_2.at(k);
				outline.push_back(current_line.start);

				intersecting_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
				distance_of_point = measure(current_line.start, intersecting_point);
				index = 0;

				for (int j=0; j<boundary_lines_1.size(); j++)
				{
					Line line = boundary_lines_1.at(j);
					Point point = current_line.get_intersecting_point(line);
					double distance = measure(current_line.start, point);
					if (distance < distance_of_point)
					{
						distance_of_point = distance;
						intersecting_point = point;
						index = j;
					}
				}

				if (!intersecting_point.isinf())
				{
					current_line = boundary_lines_1.at(index);

					outline.push_back(intersecting_point);
					outline.push_back(current_line.end);

					i = index;
					break;
				}
			}
		}
	}

	Path path = Path(outline, false);
	return path;
}

Path Pathfinder::subtract(Path path_1, Path path_2)
{
	std::vector<Line> boundary_lines_1 = path_1.get_boundary_lines();
	std::vector<Line> boundary_lines_2 = path_2.get_boundary_lines();

	std::vector<Point> outline;
	for (int i=0; i<boundary_lines_1.size(); i++)
	{// trace boundarys

		Line current_line = boundary_lines_1.at(i);
		outline.push_back(current_line.start);

		Point intersecting_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
		double distance_of_point = measure(current_line.start, intersecting_point);
		int index = 0;

		for (int j=0; j<boundary_lines_2.size(); j++)
		{// get closest intersection

			Line line = boundary_lines_2.at(j);
			Point point = current_line.get_intersecting_point(line);
			double distance = measure(current_line.start, point);
			if (distance < distance_of_point)
			{
				distance_of_point = distance;
				intersecting_point = point;
				index = j;
			}
		}

		if (!intersecting_point.isinf())
		{
			outline.push_back(intersecting_point);

			int start_index = index%boundary_lines_2.size();
			int end_index = start_index - boundary_lines_2.size();

			for (int k=start_index; k>end_index; k--)
			{
				k = k % boundary_lines_2.size();
				current_line = boundary_lines_2.at(k);

				intersecting_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
				distance_of_point = measure(current_line.start, intersecting_point);
				index = 0;

				for (int j=0; j<boundary_lines_1.size(); j++)
				{
					if (j == i) continue;

					Line line = boundary_lines_1.at(j);
					Point point = current_line.get_intersecting_point(line);
					double distance = measure(current_line.start, point);
					if (distance < distance_of_point)
					{
						distance_of_point = distance;
						intersecting_point = point;
						index = j;
					}
				}

				if (!intersecting_point.isinf())
				{
					outline.push_back(intersecting_point);
					i = index;
					break;
				}

				outline.push_back(current_line.start);
			}
		}
	}

	Path path = Path(outline, false);
	return path;
}

}

#endif /*BLINK_PI_OBJECTS_H*/