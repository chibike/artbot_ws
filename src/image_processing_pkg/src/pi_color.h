#ifndef BLINK_PI_COLOR_H
#define BLINK_PI_COLOR_H

#include <vector>
#include <math.h>
#include "Eigen/Dense"
#include "pi_objects.h"

namespace blink
{

struct RGB
{
	double r;
	double g;
	double b;
};

class RGBColor
{
public:
	RGB rgb_color;

	RGBColor();
	RGBColor(RGB color);
	RGBColor(double r, double g, double b);

	double sub(RGBColor other);
	bool equal(RGBColor other);

	void update_color(RGB color);
	void update_color(double r, double g, double b);
	std::vector<double> get_bgr(double scale);
	double measure(RGBColor other);
	double measure(RGB color);
};

RGBColor::RGBColor()
{
	update_color(0,0,0);
}

RGBColor::RGBColor(RGB color)
{
	update_color(color.r,color.b,color.g);
}

RGBColor::RGBColor(double r, double g, double b)
{
	update_color(r,g,b);
}

double RGBColor::sub(RGBColor other)
{
	return measure(other);
}

bool RGBColor::equal(RGBColor other)
{
	return ((rgb_color.r == other.rgb_color.r) && (rgb_color.g == other.rgb_color.g) && (rgb_color.b == other.rgb_color.b));
}

void RGBColor::update_color(RGB color)
{
	rgb_color.r = color.r;
	rgb_color.g = color.g;
	rgb_color.b = color.b;
}

void RGBColor::update_color(double r, double g, double b)
{
	rgb_color.r = r;
	rgb_color.g = g;
	rgb_color.b = b;
}

std::vector<double> RGBColor::get_bgr(double scale)
{
	std::vector<double> color;

	color.push_back(rgb_color.b * scale);
	color.push_back(rgb_color.g * scale);
	color.push_back(rgb_color.r * scale);

	return color;
}

double RGBColor::measure(RGBColor other)
{
	double dr = other.rgb_color.r - rgb_color.r;
	double dg = other.rgb_color.g - rgb_color.g;
	double db = other.rgb_color.b - rgb_color.b;

	double p = std::pow(dr, 2) + std::pow(dg, 2) + std::pow(db, 2);
	return std::sqrt(p);
}

double RGBColor::measure(RGB color)
{
	double dr = color.r - rgb_color.r;
	double dg = color.g - rgb_color.g;
	double db = color.b - rgb_color.b;

	double p = std::pow(dr, 2) + std::pow(dg, 2) + std::pow(db, 2);
	return std::sqrt(p);
}


class ColorCluster
{
public:

	double width;
	double height;
	double radius;
	double accept_radius;

	Point center;
	RGBColor color;
	std::vector<double> xs;
	std::vector<double> ys;

	ColorCluster();
	ColorCluster(RGBColor c, Point point, double r);

	double get_distance_from(Point point);
	void get_distance_from(ColorCluster other);
	void is_touching(ColorCluster other);
	bool append(RGBColor other_color, Point other_position);
	bool merge(ColorCluster other);
	bool check_eligibility(RGBColor other_color, Point other_position);

	void calculate();

};

ColorCluster::ColorCluster()
{
	accept_radius = 5;
	calculate();
}

ColorCluster::ColorCluster(RGBColor c, Point point, double r)
{
	accept_radius = r;
	color = c;
	
	xs.push_back(point.x);
	ys.push_back(point.y);

	calculate();
}

double ColorCluster::get_distance_from(Point point)
{
	return blink::measure(center, point);
}

void ColorCluster::calculate()
{
	double min_x = blink::minimum_element(xs);
	double max_x = blink::maximum_element(xs);

	double min_y = blink::minimum_element(ys);
	double max_y = blink::maximum_element(ys);

	width  = max_x - min_x;
	height = max_y - min_y;
	radius = std::sqrt( std::pow(width, 2) + std::pow(height, 2)) / 2.0;
	center = Point(min_x + radius, min_y + radius);
}

RGBColor simplify(RGBColor color, std::vector<RGBColor> color_array)
{
	int index = 0;
	double minimum_diff = std::abs(color.sub(color_array.at(index)));

	for (int i=0; i<color_array.size(); i++)
	{
		double diff = std::abs(color.sub(color_array.at(i)));
		if (diff < minimum_diff)
		{
			minimum_diff = diff;
			index = i;
		}
	}

	return color_array.at(index);
}

//pre-defined color constants
RGBColor RED_MAX     = RGBColor(1.0, 0.0, 0.0);
RGBColor BLUE_MAX    = RGBColor(0.0, 0.0, 1.0);
RGBColor BLACK_MAX   = RGBColor(0.0, 0.0, 0.0);
RGBColor GREEN_MAX   = RGBColor(0.0, 1.0, 0.0);
RGBColor WHITE_MAX   = RGBColor(1.0, 1.0, 1.0);
RGBColor YELLOW_MAX  = RGBColor(1.0, 1.0, 0.0);
RGBColor MAGENTA_MAX = RGBColor(1.0, 0, 1.0);
RGBColor CYAN_MAX    = RGBColor(0.0, 1.0, 1.0);

}

#endif /*BLINK_PI_COLOR_H*/