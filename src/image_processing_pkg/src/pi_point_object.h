#pragma once

#include "pi_point_header.h"

namespace blink
{

Point::Point()
{
	x = 0;
	y = 0;
	
	calculate();
}

Point::Point(const double xx, const double yy)
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
	r = sqrt(std::pow(x, 2) + std::pow(y, 2));
	theta = atan2(y,x);
	
	point.clear();
	point.reserve(2);

	point.push_back(x);
	point.push_back(y);
	point_polar.push_back(r);
	point_polar.push_back(theta);
}

Point Point::get_absolute()
{
	Point new_point = Point(std::abs(x), std::abs(y));
	return new_point;
}

template<class T>
Point Point::add(const T& val) const
{
	Point new_point = Point(x + val, y + val);
	return new_point;
}

Point Point::add(const Point& other) const
{
	Point new_point = Point(x + other.x, y + other.y);
	return new_point;
}

template<class T>
Point  Point::sub(const T& val) const
{
	Point new_point = Point(x - val, y - val);
	return new_point;
}

Point  Point::sub(const Point& other) const
{
	Point new_point = Point(x - other.x, y - other.y);
	return new_point;
}

template<class T>
Point  Point::mul(const T& val) const
{
	Point new_point = Point(x * val, y * val);
	return new_point;
}

Point  Point::mul(const Point& other) const
{
	Point new_point = Point(x * other.x, y * other.y);
	return new_point;
}

template<class T>
Point  Point::div(const T& val) const
{
	Point new_point = Point(x / val, y / val);
	return new_point;
}

Point  Point::div(const Point& other) const
{
	Point new_point = Point(x / other.x, y / other.y);
	return new_point;
}

template<class T>
Point Point::pow(const T& val) const
{
	Point new_point = Point(std::pow(x, val), std::pow(y, val));
	return new_point;
}

Point Point::pow(const Point& other) const
{
	Point new_point = Point(std::pow(x, other.x), std::pow(y, other.y));
	return new_point;
}

Point Point::log() const
{
	Point new_point = Point(std::log(x), std::log(y));
	return new_point;
}

template<class T>
Point operator+(const Point& point, const T& val)
{
	Point new_point = point.add(val);
	return new_point;
}

Point operator+(const Point& point1, const Point& point2)
{
	Point new_point = point1.add(point2);
	return new_point;
}

template<class T>
Point operator-(const Point& point, const T& val)
{
	Point new_point = point.sub(val);
	return new_point;
}

Point operator-(const Point& point1, const Point& point2)
{
	Point new_point = point1.sub(point2);
	return new_point;
}

template<class T>
Point operator*(const Point& point, const T& val)
{
	Point new_point = point.mul(val);
	return new_point;
}

Point operator*(const Point& point1, const Point& point2)
{
	Point new_point = point1.mul(point2);
	return new_point;
}

template<class T>
Point operator*(const T& val, const Point& point)
{
	Point new_point = point.mul(val);
	return new_point;
}

template<class T>
Point operator/(const Point& point, const T& val)
{
	Point new_point = point.div(val);
	return new_point;
}

template<class T>
Point operator/(const T& val, const Point& point)
{
	Point new_point = point.div(val);
	return new_point;
}

Point operator/(const Point& point1, const Point& point2)
{
	Point new_point = point1.div(point2);
	return new_point;
}

template<class T>
Point operator+=(Point& point, const T& val)
{
	Point new_point = point.add(val);
	point.x = new_point.x;
	point.y = new_point.y;
	point.calculate();
	return point;
}

Point operator+=(Point& point1, const Point& point2)
{
	
	Point new_point = point1.add(point2);
	point1.x = new_point.x;
	point1.y = new_point.y;
	point1.calculate();
	return point1;
}

template<class T>
Point operator-=(Point& point, const T& val)
{
	Point new_point = point.sub(val);
	point.x = new_point.x;
	point.y = new_point.y;
	point.calculate();
	return point;
}

Point operator-=(Point& point1, const Point& point2)
{
	
	Point new_point = point1.sub(point2);
	point1.x = new_point.x;
	point1.y = new_point.y;
	point1.calculate();
	return point1;
}

template<class T>
Point operator*=(Point& point, const T& val)
{
	Point new_point = point.mul(val);
	point.x = new_point.x;
	point.y = new_point.y;
	point.calculate();
	return point;
}

Point operator*=(Point& point1, const Point& point2)
{
	
	Point new_point = point1.mul(point2);
	point1.x = new_point.x;
	point1.y = new_point.y;
	point1.calculate();
	return point1;
}

template<class T>
Point operator/=(Point& point, const T& val)
{
	Point new_point = point.div(val);
	point.x = new_point.x;
	point.y = new_point.y;
	point.calculate();
	return point;
}

Point operator/=(Point& point1, const Point& point2)
{
	
	Point new_point = point1.div(point2);
	point1.x = new_point.x;
	point1.y = new_point.y;
	point1.calculate();
	return point1;
}


bool operator==(const Point& point1, const Point& point2)
{
	return (point1.x == point2.x) && (point1.y == point2.y);
}

bool operator!=(const Point& point1, const Point& point2)
{
	return (point1.x != point2.x) || (point1.y != point2.y);
}

bool operator<(const Point& point1, const Point& point2)
{
	return measure(Point(0,0), point1) < measure(Point(0,0), point2);
}

template<class T>
bool operator<(const T& val, const Point& point1)
{
	return (val < point1.x) && (val < point1.y);
}

template<class T>
bool operator<(const Point& point1, const T& val)
{
	return (point1.x < val) && (point1.y < val);
}

// ------------------------------------------

bool operator<=(const Point& point1, const Point& point2)
{
	return measure(Point(0,0), point1) <= measure(Point(0,0), point2);
}

template<class T>
bool operator<=(const T& val, const Point& point1)
{
	return (val <= point1.x) && (val <= point1.y);
}

template<class T>
bool operator<=(const Point& point1, const T& val)
{
	return (point1.x <= val) && (point1.y <= val);
}

// ------------------------------------------

bool operator>(const Point& point1, const Point& point2)
{
	return measure(Point(0,0), point1) > measure(Point(0,0), point2);
}

template<class T>
bool operator>(const T& val, const Point& point1)
{
	return (val > point1.x) && (val > point1.y);
}

template<class T>
bool operator>(const Point& point1, const T& val)
{
	return (point1.x > val) && (point1.y > val);
}

// ------------------------------------------

bool operator>=(const Point& point1, const Point& point2)
{
	return measure(Point(0,0), point1) >= measure(Point(0,0), point2);
}

template<class T>
bool operator>=(const T& val, const Point& point1)
{
	return (val >= point1.x) && (val >= point1.y);
}

template<class T>
bool operator>=(const Point& point1, const T& val)
{
	return (point1.x >= val) && (point1.y >= val);
}

std::ostream& operator<<(std::ostream& os, const Point& point)
{
	os << "Point(" << point.x << ", " << point.y << ")";
	return os;
}

bool Point::equal(Point other)
{
	return (std::abs(x - other.x) < 0.001 && std::abs(y - other.y) < 0.001);
}

std::vector<double> Point::get_as_polar()
{
	return point_polar;
}

bool Point::isinf()
{
	return std::isinf(x) || std::isinf(y);
}

bool Point::isnan()
{
	return std::isnan(x) || std::isnan(y);
}

}