#pragma once

#include "pi_path_header.h"

namespace blink
{

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

Path::Path(cv::Rect rect)
{
	std::vector<Point> points;

	Point point_1 = Point(rect.x,              rect.y);
	Point point_2 = Point(rect.x + rect.width, rect.y);
	Point point_3 = Point(rect.x + rect.width, rect.y + rect.height);
	Point point_4 = Point(rect.x,              rect.y + rect.height);

	points.push_back(point_1);
	points.push_back(point_2);
	points.push_back(point_3);
	points.push_back(point_4);

	data_points = points;
	closed = false;

	calculate();
}

// Path::Path(Line line)
// {
// 	std::vector<Point> points;
// 	points.push_back(line.start);
// 	points.push_back(line.end);

// 	data_points = points;
// 	closed = true;

// 	calculate();
// }

Path::Path(Point center, double radius, double angle=2.0*M_PI, unsigned int steps=32)
{
	std::vector<Point> points;

	double theta = 0.0;
	double increment = 2 * M_PI / steps;
	angle = constrainf(std::abs(angle), 0.0, 2*M_PI);
	radius = std::abs(radius);

	double x, y;
	while (theta < angle)
	{
		x = center.x + radius * std::cos(theta);
		y = center.y + radius * std::sin(theta);

		Point point = Point(x,y);
		points.push_back(point);

		theta += increment;
	}

	data_points = points;
	closed = false;

	calculate();
}

Path::Path(Point center, double rx, double ry, double angle=2.0*M_PI, unsigned int steps=32)
{
	std::vector<Point> points;

	double theta = 0.0;
	double increment = 2 * M_PI / steps;
	angle = constrainf(std::abs(angle), 0.0, 2*M_PI);
	rx = std::abs(rx);
	ry = std::abs(ry);

	double x, y;
	while (theta < angle)
	{
		x = center.x + (rx * std::cos(theta));
		y = center.y + (ry * std::sin(theta));

		Point point = Point(x,y);
		points.push_back(point);

		theta += increment;
	}

	data_points = points;
	closed = false;

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

bool Path::rotate(double angle)
{
	Point pivot = rect_info.center;
	return rotate(angle, pivot);
}

bool Path::rotate(double angle, Point pivot)
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
	return true;
}

bool Path::translate(double x, double y)
{
	data_points.clear();
	closed = false;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.translate(x, y);
		data_points.push_back(line.start);
	}

	calculate();
	return true;
}

bool Path::shear(double x, double y)
{
	data_points.clear();
	closed = false;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.shear(x, y);
		data_points.push_back(line.start);
	}

	calculate();
	return true;
}

bool Path::scale(double x, double y)
{
	data_points.clear();
	closed = false;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.scale(x, y);
		data_points.push_back(line.start);
	}

	calculate();
	return true;
}

bool Path::scale(double x, double y, Point about_point)
{
	data_points.clear();
	closed = false;

	double tx = about_point.x / 2;
	double ty = about_point.y / 2;

	for (int i=0; i<boundary_lines.size(); i++)
	{
		Line line = boundary_lines.at(i);
		line.scale(x, y);
		line.translate(tx, ty);
		data_points.push_back(line.start);
	}

	calculate();
	return true;
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

	// cout << "--- line is within ---" << endl;
	// cout << "y = " << (double)line.start.y << endl;
	// cout << "start, end = " << (double)line.start.x << " - " << (double)line.end.x << endl;
	// cout << "length = " << (double)line.get_length() << endl;
	// cout << "--- intersection points ---" << endl;
	// cout << "\t";
	// cout << "points = ";
	// for (int i=0; i<points.size(); i++)
	// {
	// 	Point p = points.at(i);
	// 	cout << "px_" << ((double) p.x) << ", ";
	// }
	// cout << endl;


	if (points.size() == 1)
	{
		if (is_touching(line.start) && !(line.start.equal(points.at(0))))
		{
			// cout << "01 is_within: " << "true";
			// cout << " d" << points.at(0).x - line.start.x << " d";
			// cout << endl;
			return true;
		}
		else if (is_touching(line.end) && !(line.end.equal(points.at(0))))
		{
			// cout << "02 is_within: " << "true";
			// cout << endl;
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

		// cout << " dx_" << dx_0 << "__" << dx_1 << "_";

		return (dx_0 >= -0.0001 && dx_1 >= -0.0001);
	}

	if (is_within(line.start)  || is_within(line.end))
	{
		// cout << "03a is_within: " << "true";
		// cout << endl;
		return true;
	}
	else if ( is_touching(line.start) && is_touching(line.end) )
	{
		// cout << "03c is_within: " << "true";
		// cout << endl;
		return true;
	}
	else
	{
		// cout << "04 is_within: " << "false";
		// cout << endl;
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

void Path::mask(cv::Mat image_in, cv::Mat &image_out, bool override_image_out=true, cv::Scalar empty_color=cv::Scalar(0,0,0), int translation_x=0, int translation_y=0)
{
	if (override_image_out || image_out.empty())
	{
		image_out = image_in.clone();
		image_out.setTo(empty_color);
	}

	for (int row=0; row<image_in.rows; row++)
	{
		for (int col=0; col<image_in.cols; col++)
		{
			// Point point = Point(col, row);
			// if (is_within(point))
			// {
			// 	int new_row = row + translation_x; int new_col = col + translation_y;

			// 	if (new_col >= 0 && new_row >= 0 && new_col < image_out.cols && new_row < image_out.rows)
			// 	{
			// 		image_out.at<cv::Vec3b>(new_row, new_col) = image_in.at<cv::Vec3b>(row, col);
			// 	}
			// }
		}
	}
}

std::string Path::get_as_string() const
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

double Path::calculate_signed_area()
{
	double var = 0;
	for (int i=0; i<data_points.size(); i++)
	{
		int next = (i + 1)  % data_points.size();
		var += (xs.at(i) * ys.at(next)) - (xs.at(next) * ys.at(i));
	}

	return 0.5 * var;
}

Point Path::calculate_centroid()
{ // REF: https://en.wikipedia.org/wiki/Centroid
	// for a non-self-intersecting path

	const double scale = 1 / (6 * calculate_signed_area());
	double cx = 0;
	double cy = 0;

	for (int i=0; i<data_points.size(); i++)
	{
		int next = (i + 1)  % data_points.size();
		cx += (xs.at(i) + xs.at(next)) * ((xs.at(i)*ys.at(next)) - (xs.at(next)*ys.at(i)));
		cy += (ys.at(i) + ys.at(next)) * ((xs.at(i)*ys.at(next)) - (xs.at(next)*ys.at(i)));
	}

	cx = cx * scale;
	cy = cy * scale;

	return Point(cx, cy);
}

double Path::fuzzy_compare(Path other)
{
	// based on the difference in the number of sizes
	double sides_diff = std::abs(boundary_lines.size() - other.boundary_lines.size());

	// based on the distance between the centroid and geometric center of the path
	double controid_diff = std::abs( measure(rect_info.center, calculate_centroid()) - measure(other.rect_info.center, other.calculate_centroid()) );

	return 0.0;
}

std::ostream& operator<<(std::ostream& os, const Path& path)
{
	os << path.get_as_string();
	return os;
}

}