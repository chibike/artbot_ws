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

class Path
{
public:
	std::vector<Point> data_points;
	std::vector<double> xs;
	std::vector<double> ys;
	std::vector<Line> boundary_lines;

	bool closed;
	RectInfo rect_info;

	Path(cv::Rect rect);
	Path(std::vector<Point> raw_point_data, bool closed);
	Path(std::vector<cv::Point> raw_point_data, bool closed);
	Path(Point center, double radius, double angle, unsigned int steps);
	Path(Point center, double rx, double ry, double angle, unsigned int steps);

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
	void mask(cv::Mat image_in, cv::Mat &image_out, bool override_image_out, cv::Scalar empty_color, int translation_x, int translation_y);
	void draw(cv::Mat image, cv::Scalar color);
	void draw(cv::Mat image, cv::Scalar color, int stroke_width, int line_type, int shift);

	// reflect()
	bool translate(double x, double y);
	bool shear(double x, double y);
	bool scale(double x, double y);
	bool scale(double x, double y, Point about_point);
	bool rotate(double angle);
	bool rotate(double angle, Point pivot);

	double fuzzy_compare(Path other);
	Point  calculate_centroid();
	double calculate_signed_area();

	// void filter_points(double min_distance);
	// void filter_points(double min_distance, double max_distance);

	friend std::ostream& operator<<(std::ostream& os, const Path& path);
	std::string get_as_string() const;

	void calculate();
	static bool _filter_x_function(Point current, Point next)
	{
		return current.x < next.x;
	}
};

}