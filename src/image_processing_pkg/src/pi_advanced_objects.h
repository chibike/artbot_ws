
#pragma once

#include <cfloat>
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include "Eigen/Dense"
#include "pi_objects.h"
#include "pi_arithmetic.h"
#include "opencv2/opencv.hpp"

namespace  blink
{

template<class T>
double measure_segment(T& segment, Point& start_point, Point& end_point, const unsigned int max_depth, unsigned int depth = 0) // , const unsigned int& depth=0
{
	/*
	  This function recursively approximates the length of the segment uing straight lines
	  inspired / (originally written) by : https://raw.githubusercontent.com/regebro/svg.path/master/src/svg/path/path.py
	*/ 

	Line approx_seg = Line(start_point, end_point);
	Point approx_mid_point = approx_seg.get_midpoint();

	Point mid_point = segment.get_point(0.5);
	Line first_half = Line(start_point, mid_point);
	Line second_half = Line(mid_point, end_point);

	double approx_seg_length = approx_seg.get_length();
	double approx_curve_length = std::abs(first_half.get_length()) + std::abs(second_half.get_length());

	if ((approx_curve_length - approx_seg_length > DBL_EPSILON) && (depth < max_depth))
	{
		depth++;
		return measure_segment(segment, start_point, approx_mid_point, max_depth, depth) + measure_segment(segment, approx_mid_point, end_point, max_depth, depth);
	}
	

	return approx_curve_length;
}

class CubicBezier
{
public:
	Point start;
	Point end;
	Point control1;
	Point control2;

	bool is_smooth_from(CubicBezier &bezier);
	
	template<class T>
	bool is_smooth_from(T &bezier);

	double get_length(unsigned int max_depth);
	Point get_point(const double position);

	friend ostream& operator<<(ostream& os, const CubicBezier& bezier);

	CubicBezier();
	CubicBezier(Point start, Point end, Point control1, Point control2);
};

CubicBezier::CubicBezier()
{
}

CubicBezier::CubicBezier(Point start, Point end, Point control1, Point control2)
{
	this->start = start;
	this->end = end;
	this->control1 = control1;
	this->control2 = control2;
}

ostream& operator<<(ostream& os, const CubicBezier& bezier)
{
	os << "CubicBezier(start=" << bezier.start << ", end=" << bezier.end << ", control1=" << bezier.control1 << ", control2=" << bezier.control2 << ")";
	return os;
}

bool CubicBezier::is_smooth_from(CubicBezier &bezier)
{
	return (this->start == bezier.end) && (measure(this->control1, this->start) == measure(bezier.end, bezier.control2));
}

template<class T>
bool CubicBezier::is_smooth_from(T &bezier)
{
	return this->control1 == this->start;
}

Point CubicBezier::get_point(const double position)
{
	Point a = std::pow(1.0 - position,  3) * this->start;
	Point b = 3.0 * std::pow(1.0 - position, 2) * position * this->control1;
	Point c = 3.0 * (1.0 - position) * std::pow(position, 2) * this->control2;
	Point d = std::pow(position, 3) * this->end;

	return a + b + c + d;
}

double CubicBezier::get_length(unsigned int max_depth=5)
{
	/*
	  This implementation is directly ported from https://raw.githubusercontent.com/regebro/svg.path/master/src/svg/path/path.py
	*/
	return measure_segment(*this, this->start, this->end, 5, 0);
}

// -----------------------------------------------


class QuadraticBezier
{
public:
	Point start;
	Point end;
	Point control;

	template<class T>
	bool is_smooth_from(T &bezier);

	double get_length(unsigned int max_depth);
	bool is_smooth_from(QuadraticBezier &bezier);
	Point get_point(double position); // returns a point the position on the curve; position ranges from 0..1

	QuadraticBezier();
	QuadraticBezier(Point start, Point end, Point control);

	friend ostream& operator<<(ostream& os, const QuadraticBezier& bezier);
};

QuadraticBezier::QuadraticBezier()
{
}

QuadraticBezier::QuadraticBezier(Point start, Point end, Point control)
{
	this->start = start;
	this->end = end;
	this->control = control;
}

ostream& operator<<(ostream& os, const QuadraticBezier& bezier)
{
	os << "QuadraticBezier(start=" << bezier.start << ", end=" << bezier.end << ", control=" << bezier.control << ")";
	return os;
}

bool QuadraticBezier::is_smooth_from(QuadraticBezier &bezier)
{
	return (this->start == bezier.end) && (measure(this->control, this->start) == measure(bezier.end, bezier.control));
}

template<class T>
bool QuadraticBezier::is_smooth_from(T &bezier)
{
	return this->control == this->start;
}

Point QuadraticBezier::get_point(double position)
{
	Point a = std::pow(1 - position, 2) * this->start;
	Point b = 2 * (1 - position) * position * control;
	Point c = std::pow(position, 2) * end;
	return a + b + c;
}

double QuadraticBezier::get_length(unsigned int max_depth=5)
{
	// Point a = this->start - 2*this->control + this->end;
	// Point b = 2*(this->control - this->start);

	// Point abs_a = a.get_absolute();
	// Point abs_b = b.get_absolute();

	// // TODO: this has to be a dot product
	// double a_dot_b = (a.x * b.x) + (a.y * b.y);
	
	// Point s;

	// if (abs_a < DBL_EPSILON)
	// {
	// 	s = abs_b;
	// }
	// else if (((abs_a * abs_b) + a_dot_b).get_absolute() < DBL_EPSILON)
	// {
	// 	Point k = abs_b / abs_a;
	// 	if (k <= 2)
	// 	{
	// 		s = abs_b - abs_a;
	// 	}
	// 	else
	// 	{
	// 		s = abs_a * ((k.pow(2) / 2) - k + 1);
	// 	}
	// }
	// else
	// {
	// 	/*
	// 		For an explanation of this case, see
	// 		http://www.malczak.info/blog/quadratic-bezier-curve-length/
	// 	*/
	// 	Point A = 4 * a.pow(2);
	// 	Point B = 4 * a_dot_b;
	// 	Point C = b.pow(2);

	// 	/* Sabc = 2 * sqrt(A + B + C) */
	// 	Point abc = A + B + C;
	// 	Point Sabc = 2 * abc.pow(0.5);

	// 	Point A2 = A.pow(0.5);
	// 	Point A32 = 2 * A * A2;
	// 	Point C2 = 2 * C.pow(2);
	// 	Point BA = B / A2;

	// 	s = (A32 * Sabc + A2 * B * (Sabc - C2) + (4 * C * A - B.pow(2)) * ((2*A2 + BA + Sabc) / (BA + C2)).log()) / (4 * A32);
	// }

	// return s;
	return measure_segment(*this, this->start, this->end, 5, 0);
}

enum BezierType {NONE, CUBIC, QUADRATIC};

class Bezier
{
private:
	CubicBezier _cubic_curve;
	QuadraticBezier _quadratic_curve;

public:
	Bezier();

	Bezier(CubicBezier &curve);
	Bezier(QuadraticBezier &curve);

	BezierType type;

	BezierType get_type() const;
	Point get_start_point();
	Point get_end_point();

	bool get_curve(CubicBezier& curve_out);
	bool get_curve(QuadraticBezier& curve_out);


	template<class T>
	bool is_smooth_from(T &next_curve);
	bool is_smooth_from(Bezier &next_curve);
	bool is_smooth_from(CubicBezier &next_curve);
	bool is_smooth_from(QuadraticBezier &next_curve);

	double get_length(unsigned int max_depth);
	Point get_point(double position); // returns a point the position on the curve; position ranges from 0..1
};

Bezier::Bezier()
{
	type = NONE;
}

Bezier::Bezier(CubicBezier &curve)
{
	type = CUBIC;
	_cubic_curve = curve;
}

Bezier::Bezier(QuadraticBezier &curve)
{
	type = QUADRATIC;
	_quadratic_curve = curve;
}

double Bezier::get_length(unsigned int max_depth)
{
	if (type == CUBIC)
	{
		return _cubic_curve.get_length(max_depth);// ((CubicBezier*) curve)->get_length(max_depth);
	}
	else if (type == QUADRATIC)
	{
		return _quadratic_curve.get_length(max_depth);// ((QuadraticBezier*) curve)->get_length(max_depth);
	}

	return 0.0;
}

BezierType Bezier::get_type() const
{
	return type;
}

bool Bezier::get_curve(CubicBezier& curve_out)
{
	curve_out = _cubic_curve;
	return true;
}

bool Bezier::get_curve(QuadraticBezier& curve_out)
{
	curve_out = _quadratic_curve;
	return true;
}

bool Bezier::is_smooth_from(Bezier &next_curve)
{
	if (type == CUBIC && next_curve.type == CUBIC)
	{
		CubicBezier next_cubic_curve;
		next_curve.get_curve(next_cubic_curve);
		return (_cubic_curve.end == next_cubic_curve.start) && ((_cubic_curve.control1 - _cubic_curve.end) == (next_cubic_curve.control1 - next_cubic_curve.start));
	}
	else if (type == QUADRATIC && next_curve.type == QUADRATIC)
	{
		QuadraticBezier next_quadratic_curve;
		next_curve.get_curve(next_quadratic_curve);
		return (_quadratic_curve.end == next_quadratic_curve.start) && ((_quadratic_curve.control - _quadratic_curve.end) == (next_quadratic_curve.control == next_quadratic_curve.start));
	}
	else
	{
		switch(type)
		{
			case CUBIC: return _cubic_curve.end == next_curve.get_start_point(); break;
			case QUADRATIC: return _quadratic_curve.end == next_curve.get_start_point(); break;
		}
	}
}

bool Bezier::is_smooth_from(QuadraticBezier &next_curve)
{
	if (type == QUADRATIC)
	{
		return (_quadratic_curve.end == next_curve.start) && ((_quadratic_curve.control - _quadratic_curve.end) == (next_curve.control == next_curve.start));
	}
	else
	{
		switch(type)
		{
			case CUBIC: return _cubic_curve.end == next_curve.start; break;
			case QUADRATIC: return _quadratic_curve.end == next_curve.start; break;
		}
	}
}

bool Bezier::is_smooth_from(CubicBezier &next_curve)
{
	if (type == CUBIC)
	{
		return (_cubic_curve.end == next_curve.start) && ((_cubic_curve.control1 - _cubic_curve.end) == (next_curve.control1 - next_curve.start));
	}
	else
	{
		switch(type)
		{
			case CUBIC: return _cubic_curve.end == next_curve.start; break;
			case QUADRATIC: return _quadratic_curve.end == next_curve.start; break;
		}
	}
}

template<class T>
bool Bezier::is_smooth_from(T &next_curve)
{
	switch(type)
	{
		switch(type)
		{
			case CUBIC: return _cubic_curve.end == next_curve.start; break;
			case QUADRATIC: return _quadratic_curve.end == next_curve.start; break;
		}
	}
}

Point Bezier::get_start_point()
{
	if (type == CUBIC)
	{
		return _cubic_curve.start;
	}
	else if (type == QUADRATIC)
	{
		return _quadratic_curve.start;
	}

	Point new_point;
	new_point.set_inf();

	return new_point;
}

Point Bezier::get_end_point()
{
	if (type == CUBIC)
	{
		return _cubic_curve.end;
	}
	else if (type == QUADRATIC)
	{
		return _quadratic_curve.end;
	}

	Point new_point;
	new_point.set_inf();

	return new_point;
}

Point Bezier::get_point(double position)
{
	if (type == CUBIC)
	{
		return _cubic_curve.get_point(position);
	}
	else if (type == QUADRATIC)
	{
		return _quadratic_curve.get_point(position);
	}

	Point new_point;
	new_point.set_inf();

	return new_point;
}

class Arc
{
private:
	Point center;
	double theta;
	double delta;

	void _parameterize();

public:
	Point start;
	Point end;

	double radius_x;
	double radius_y;
	double rotation;
	bool arc;
	bool sweep;

	Arc(Point start, Point end, double radius_x, double radius_y, double rotation, bool arc, bool sweep);
	Point get_point(double position); // returns a point the position on the curve; position ranges from 0..1
	double get_length(double max_depth);

	friend ostream& operator<<(ostream& os, const Arc& bezier);
};

Arc::Arc(Point start, Point end, double radius_x, double radius_y, double rotation, bool arc, bool sweep)
{
	this->start = start;
	this->end = end;
	this->radius_x = radius_x;
	this->radius_y = radius_y;
	this->rotation = rotation;
	this->arc = arc;
	this->sweep = sweep;

	_parameterize();
}

ostream& operator<<(ostream& os, const Arc& arc)
{
	os << "Arc(start=" << arc.start << ", end=" << arc.end << ", radius_x=" << arc.radius_x << ", radius_y=" << arc.radius_y << ", rotation=" << arc.rotation << ", arc=" << arc.arc << ", sweep=" << arc.sweep << ")";
	return os;
}


void Arc::_parameterize()
{
	/*
		Conversion from endpoint to center parameterization
        http://www.w3.org/TR/SVG/implnote.html#ArcImplementationNotes
	*/

	double cosr = std::cos(this->rotation);
	double sinr = std::sin(this->rotation);
	
	double dx = ( this->start.x - this->end.x ) / 2.0;
	double dy = ( this->start.y - this->end.y ) / 2.0;
	
	double xlprim = (cosr * dx) + (sinr * dy);
	double ylprim = (-1.0 * sinr * dx) + (cosr * dy);
	
	double xlprim_sq = xlprim * xlprim;
	double ylprim_sq = ylprim * ylprim;

	double rx = radius_x;
	double ry = radius_y;

	double rx_sq = rx * rx;
	double ry_sq = ry * ry;

	// correct out of range radii
	double radius_check = (xlprim_sq / rx_sq) + (ylprim_sq / ry_sq);

	if (radius_check > 1)
	{
		rx *= std::sqrt(radius_check);
		ry *= std::sqrt(radius_check);

		rx_sq = rx * rx;
		ry_sq = ry * ry;
	}

	double t1 = rx_sq * ylprim_sq;
	double t2 = ry_sq * xlprim_sq;
	double c = std::sqrt(std::abs(((rx_sq * ry_sq) - t1 - t2) / (t1 + t2)));

	if (this->arc == this->sweep)
	{
		c = -1.0 * c;
	}

	double cxprim = c * rx * ylprim / ry;
	double cyprim = -c * ry * xlprim / rx;

	double cx = ((cosr * cxprim) - (sinr * cyprim)) + ((this->start.x + this->end.x) / 2.0);
	double cy = ((sinr * cxprim) + (cosr * cyprim)) + ((this->start.y + this->end.y) / 2.0);
	this->center = Point(cx, cy);

	double ux = (xlprim - cxprim) / rx;
	double uy = (ylprim - cyprim) / ry;
	double vx = (-xlprim - cxprim) / rx;
	double vy = (-ylprim - cyprim) / ry;
	double n = std::sqrt(std::pow(ux, 2) + std::pow(uy, 2));
	double p = ux;
	double theta = std::acos(p / n);
	if (uy < 0)
	{
		theta = -theta;
	}

	this->theta = std::fmod(theta, 2.0*M_PI); //theta % (2.0*M_PI);

	n = std::sqrt((std::pow(ux, 2) + std::pow(uy, 2)) * (std::pow(vx, 2) + std::pow(vy, 2)));
	p = (ux * vx) + (uy * vy);
	double d = p / n;

	//	# In certain cases the above calculation can through inaccuracies
	//	# become just slightly out of range, f ex -1.0000000000000002.

	if (d > 1.0)
	{
		d = 1.0;
	}
	else if (d < -1.0)
	{
		d = -1.0;
	}

	double delta = std::acos(d);
	if (((ux * vy) - (uy * vx)) < 0)
	{
		delta = -1.0 * delta;
	}

	this->delta = std::fmod(delta, 2.0*M_PI); //delta % (2.0*M_PI);

	if (!this->sweep)
	{
		this->delta -= 2.0*M_PI;
	}
}

Point Arc::get_point(double position)
{
	double angle = this->theta + (this->delta * position);
	double cosr = std::cos(this->rotation);
	double sinr = std::sin(this->rotation);

	double x = (cosr * std::cos(angle) * this->radius_x - sinr * std::sin(angle) * this->radius_y + this->center.x);
	double y = (sinr * std::cos(angle) * this->radius_x + cosr * std::sin(angle) * this->radius_y + this->center.y);

	Point new_point = Point(x, y);
	return new_point;
}

double Arc::get_length(double max_depth=5)
{
	/*
		The length of an elliptical arc segment requires numerical
		integration, and in that case it's simpler to just do a geometric
		approximation, as for cubic bezier curves.
	*/

	return measure_segment(*this, this->start, this->end, 5, 0);
}




}