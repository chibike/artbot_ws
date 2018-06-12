#pragma once

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include "Eigen/Dense"
#include "pi_objects.h"
#include "pi_arithmetic.h"
#include "opencv2/opencv.hpp"
#include "pi_advanced_objects.h"

#define BLINK_OBJECT_PARSE_RESOLUTION 0.001

#define MODE_DEBUG 0
#define MODE_RELEASE 1
#define SKETCH_SHEET_RUN_MODE MODE_RELEASE

namespace blink
{

enum CurveType
{
	ARC,
	BEZIER,
	LINE
};


class SketchSheet
{
private:
	Bezier _last_curve;
	CurveType _last_object_type;

	Point _current_position;
	Point _previous_position;
	Point _previous_control_point;

	bool _is_sketching;
	bool _has_previous_control_point;
	std::vector<Path> _paths;
	std::vector<Point> _points;

public:
	double width;
	double height;

	SketchSheet();
	SketchSheet(double width, double height);

	void reserve(unsigned int howmany);

	Point get_current_position() const;
	Point get_last_control_position() const;

	template<class T>
	bool is_transition_smooth(T& next_curve) const;

	bool start_sketch(const unsigned int reserve_count);
	bool stop_sketch(bool closed);
	bool set_sheet_size(double width, double height);
	bool derieve_sheet_size(int top_padding, int bottom_padding, int left_padding, int right_padding, double scale);

	bool moveto(Point position, bool relative);
	bool lineto(Point position, bool relative);
	bool vertical_lineto(double new_y, bool relative);
	bool horizontal_lineto(double new_x, bool relative);
	
	bool quadratic_curveto(Point end_position, double steps, bool relative);
	bool quadratic_curveto(Point end_position, Point control_position, double steps, bool relative);
	bool cubic_curveto(Point end_position, Point control_position2, double steps, bool relative);
	bool cubic_curveto(Point end_position, Point control_position1, Point control_position2, double steps, bool relative);
	bool arcto(Point end_position, double radius_x, double radius_y, double rotation, bool arc, bool sweep, double steps, bool relative);

	bool add_circle(Point center, double radius, double angle, unsigned int steps);
	bool add_ellipse(Point center, double rx, double ry, double angle, unsigned int steps);
	bool add_line(Point start, Point end);
	bool add_rect(cv::Rect rect);
	bool add_rect(Point start, double width, double height);
	bool add_path(std::vector<Point> points, bool closed);

	bool get_sketches_as_paths(std::vector<Path> &paths_out);
	bool get_sketches_as_cv_mat(cv::Mat &frame_out, int width, int height, cv::Scalar bg_color, cv::Scalar fg_color);
};

SketchSheet::SketchSheet()
{
	this->width = 0;
	this->height = 0;
	this->_is_sketching = false;
	this->_has_previous_control_point = false;
}

SketchSheet::SketchSheet(double width, double height)
{
	this->width = width;
	this->height = height;
	this->_is_sketching = false;
	this->_has_previous_control_point = false;
}

void SketchSheet::reserve(unsigned int howmany)
{
	this->_paths.reserve(howmany);
}

Point SketchSheet::get_current_position() const
{
	return this->_current_position;
}

Point SketchSheet::get_last_control_position() const
{
	if (this->_has_previous_control_point == false)
	{
		return this->get_current_position();
	}
	else
	{
		return this->_previous_control_point;
	}
}

template<class T>
bool SketchSheet::is_transition_smooth(T& next_curve) const
{
	return this->_has_previous_control_point;
}

bool SketchSheet::set_sheet_size(double width, double height)
{
	this->width = width;
	this->height = height;

	return true;
}

bool SketchSheet::derieve_sheet_size(int top_padding=1, int bottom_padding=1, int left_padding=1, int right_padding=1, double scale=2.0)
{
	if (this->_paths.size() < 1)
	{
		return false;
	}

	double min_x = DBL_MAX;
	double min_y = DBL_MAX;
	double max_width = 0;
	double max_height = 0;

	for (int i=0; i<this->_paths.size(); i++)
	{
		if (scale != 1)
		{
			this->_paths.at(i).scale(scale, scale);
		}

		Path path = this->_paths.at(i);
		
		RectInfo info = path.get_rect_info();
		Point path_top_left = info.top_left;

		min_x = std::min(min_x, path_top_left.x);
		min_y = std::min(min_y, path_top_left.y);
		max_width = std::max(max_width, info.width + path_top_left.x);
		max_height = std::max(max_height, info.height + path_top_left.y);
	}

	this->width = (max_width - min_x) + left_padding + right_padding;
	this->height = (max_height - min_y) + top_padding + bottom_padding;

	if (min_x != 0 || min_y != 0)
	{
		double dx = -1 * (min_x - left_padding);
		double dy = -1 * (min_y - top_padding);

		for (int i=0; i<this->_paths.size(); i++)
		{
			this->_paths.at(i).translate(dx, dy);
		}
	}

	return true;
}

bool SketchSheet::moveto(Point position, bool relative=false)
{
	if (relative)
	{
		position = this->_current_position + position;
	}

	this->_previous_position = this->_current_position;
	this->_current_position = position;

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "MOVE_TO; frm: " << this->_previous_position << ", to: " << this->_current_position << ", rel: " << relative << endl;
	#endif


	return true;
}

bool SketchSheet::lineto(Point position, bool relative=false)
{
	if (!this->_is_sketching)
	{
		return false;
	}
	
	if (relative)
	{
		position = this->_current_position + position;
	}

	if (position.isnan())
	{
		return false;
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "LINE_TO; position: " << position << ", rel: " << relative << endl;
	#endif

	this->_points.push_back(position);
	this->moveto(position);

	_last_object_type = LINE;
	return true;
}

bool SketchSheet::vertical_lineto(double new_y, bool relative=false)
{
	if (relative)
	{
		new_y = this->_current_position.y + new_y;
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "VERT_TO; new_y: " << new_y << ", rel: " << relative << endl;
	#endif


	if (!this->_is_sketching)
	{
		return false;
	}

	Point position = Point(this->_current_position.x, new_y);
	
	_last_object_type = LINE;
	return this->lineto(position);
}

bool SketchSheet::horizontal_lineto(double new_x, bool relative=false)
{
	if (relative)
	{
		new_x = this->_current_position.x + new_x;
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "HORZ_TO; new_x: " << new_x << ", rel: " << relative << endl;
	#endif

	if (!this->_is_sketching)
	{
		return false;
	}

	Point position = Point(new_x, this->_current_position.y);
	
	_last_object_type = LINE;
	return this->lineto(position);
}

bool SketchSheet::quadratic_curveto(Point end_position, double steps=BLINK_OBJECT_PARSE_RESOLUTION, bool relative=false)
{
	Point control_position;

	if ( _last_object_type == BEZIER )
	{
		double cx, cy;
		cx = (2.0 * this->_current_position.x) - this->_previous_control_point.x;
		cy = (2.0 * this->_current_position.y) - this->_previous_control_point.y;
		control_position = Point(cx,cy);
	}
	else
	{
		control_position = this->_current_position;
	}

	if (relative)
	{
		end_position = this->_current_position + end_position;
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "POLY QUAD CURVE" << endl;
		// return lineto(end_position, relative);
	#endif

	return quadratic_curveto(end_position, control_position, steps, false);
}

bool SketchSheet::quadratic_curveto(Point end_position, Point control_position, double steps=BLINK_OBJECT_PARSE_RESOLUTION, bool relative=false)
{
	if (!this->_is_sketching)
	{
		return false;
	}
	
	if (relative)
	{
		end_position = this->_current_position + end_position;
		control_position = this->_current_position + control_position;
	}

	QuadraticBezier curve = QuadraticBezier(this->_current_position, end_position, control_position);

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "QUAD_TO; curve: " << curve << ", rel: " << relative << endl;
		// return lineto(end_position, relative);
	#endif

	for (double i=steps; i<1.0; i+=steps)
	{
		if (!this->lineto(curve.get_point(i)))
		{
			return false;
		}
	}

	if (!this->lineto(end_position))
	{
		return false;
	}

	this->_previous_control_point = control_position;
	this->_has_previous_control_point = true;
	_last_curve = Bezier(curve);
	_last_object_type = BEZIER;
	return true;
}

bool SketchSheet::cubic_curveto(Point end_position, Point control_position2, double steps=BLINK_OBJECT_PARSE_RESOLUTION, bool relative=false)
{
	if (relative)
	{
		end_position = this->_current_position + end_position;
		control_position2 = this->_current_position + control_position2;
	}
	
	Point control_position1;
	if ( _last_object_type == BEZIER )
	{
		double cx, cy;
		cx = (2.0 * this->_current_position.x) - this->_previous_control_point.x;
		cy = (2.0 * this->_current_position.y) - this->_previous_control_point.y;
		control_position1 = Point(cx,cy);
	}
	else
	{
		control_position1 = this->_current_position;
	}


	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "POLY CUBE CURVE" << endl;
		// return lineto(end_position, false);
	#endif

	return cubic_curveto(end_position, control_position1, control_position2, steps, false);
}

bool SketchSheet::cubic_curveto(Point end_position, Point control_position1, Point control_position2, double steps=BLINK_OBJECT_PARSE_RESOLUTION, bool relative=false)
{
	if (!this->_is_sketching)
	{
		return false;
	}

	if (relative)
	{
		end_position = this->_current_position + end_position;
		control_position1 = this->_current_position + control_position1;
		control_position2 = this->_current_position + control_position2;
	}

	CubicBezier curve = CubicBezier(this->_current_position, end_position, control_position1, control_position2);

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "CUBE_TO; curve: " << curve << ", rel: " << relative << endl;
	#endif

	for (double i=steps; i<1.0; i+=steps)
	{
		if (!this->lineto(curve.get_point(i)))
		{
			return false;
		}
	}

	if (!this->lineto(end_position))
	{
		return false;
	}

	this->_previous_control_point = control_position2;
	this->_has_previous_control_point = true;
	_last_curve = Bezier(curve);
	_last_object_type = BEZIER;
	return true;
}

bool SketchSheet::arcto(Point end_position, double radius_x, double radius_y, double rotation, bool arc, bool sweep, double steps=BLINK_OBJECT_PARSE_RESOLUTION, bool relative=false)
{
	if (!this->_is_sketching)
	{
		return false;
	}
	
	if (relative)
	{
		end_position = this->_current_position + end_position;
		radius_x = this->_current_position.x + radius_x;
		radius_y = this->_current_position.y + radius_y;
	}

	Arc curve = Arc(this->_current_position, end_position, radius_x, radius_y, rotation, arc, sweep);
	_last_object_type = ARC;
	
	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "ARC_TO; curve: " << curve << ", rel: " << relative << endl;
	#endif
	return lineto(end_position);

	for (double i=steps; i<1.0; i+=steps)
	{
		if (!this->lineto(curve.get_point(i)))
		{
			return false;
		}
	}

	if (!this->lineto(end_position))
	{
		return false;
	}

	return true;
}

bool SketchSheet::start_sketch(const unsigned int reserve_count=20)
{
	if (this->_is_sketching)
	{
		return false;
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "STARTING NEW SKETCH" << endl;
	#endif

	this->_points.clear();
	this->_is_sketching = true;
	this->_points.reserve(reserve_count);

	//this->_points.push_back(this->_current_position);
	this->_has_previous_control_point = false;
	return true;
}

bool SketchSheet::stop_sketch(bool closed=true)
{
	if (this->_points.size() > 0)
	{
		if (!closed)
		{
			this->lineto(this->_points.at(0));
		}

		this->add_path(this->_points, true);
	}

	#if SKETCH_SHEET_RUN_MODE == MODE_DEBUG
		std::cout << "STOPING CURRENT SKETCH" << endl;
	#endif

	this->_is_sketching = false;
	this->_has_previous_control_point = false;
	return true;
}


bool SketchSheet::add_circle(Point center, double radius, double angle, unsigned int steps)
{
	Path path = Path(center, radius, angle, steps);
	this->_paths.push_back(path);
	return true;
}

bool SketchSheet::add_ellipse(Point center, double rx, double ry, double angle, unsigned int steps)
{
	Path path = Path(center, rx, ry, angle, steps);
	this->_paths.push_back(path);
	return true;
}

bool SketchSheet::add_line(Point start, Point end)
{
	std::vector<Point> points = {start, end};
	
	Path path = Path(points, true);
	this->_paths.push_back(path);
	return true;
}

bool SketchSheet::add_rect(cv::Rect rect)
{
	Path path = Path(rect);
	this->_paths.push_back(path);
	return true;
}

bool SketchSheet::add_rect(Point start, double width, double height)
{
	cv::Rect rect = cv::Rect(start.x, start.y, width, height);
	return this->add_rect(rect);
}

bool SketchSheet::add_path(std::vector<Point> points, bool closed)
{
	if (points.size() <= 1)
	{
		return false;
	}

	Path path = Path(points, closed);
	this->_paths.push_back(path);
	return true;
}

bool SketchSheet::get_sketches_as_paths(std::vector<Path> &paths_out)
{
	paths_out.insert(paths_out.end(), this->_paths.begin(), this->_paths.end());
	return true;
}

bool SketchSheet::get_sketches_as_cv_mat(cv::Mat &frame_out, int width, int height, cv::Scalar bg_color, cv::Scalar fg_color)
{
	// set rows (height) and cols (width); #imagecoordinates
	frame_out = cv::Mat(height, width, CV_8UC3, bg_color);

	for (int i=0; i<this->_paths.size(); i++)
	{
		Path path = this->_paths.at(i);
		path.draw(frame_out, fg_color);
	}

	return true;
}

}

// if ( _sketch_sheet.is_joint_smooth() )
// {
// 	Point last_control_point = _sketch_sheet.get_last_control_position();
// 	cx = (2.0 * current_pos.x) - last_control_point.x;
// 	cy = (2.0 * current_pos.y) - last_control_point.y;
// }
// else
// {
// 	cx = current_pos.x;
// 	cy = current_pos.y;
// }
