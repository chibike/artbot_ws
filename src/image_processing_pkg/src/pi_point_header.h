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

class Point
{
public:
	double          x;
	double          y;
	double          r;
	double          theta;
	std::vector<double>  point;
	std::vector<double>  point_polar;

	Point();
	Point(const double xx, const double yy);

	bool            isinf();
	bool            isnan();

	template<class T>
	Point add(const T& val) const;
	Point add(const Point& other) const;
	
	template<class T>
	Point sub(const T& val) const;
	Point sub(const Point& other) const;

	template<class T>
	Point mul(const T& val) const;
	Point mul(const Point& other) const;

	template<class T>
	Point div(const T& val) const;
	Point div(const Point& other) const;

	template<class T>
	Point pow(const T& val) const;
	Point pow(const Point& other) const;

	Point log() const;

	friend Point operator+(const Point& point1, const Point& point2);
	template<class T>
	friend Point operator+(const Point& point, const T& val);
	
	friend Point operator-(const Point& point1, const Point& point2);
	template<class T>
	friend Point operator-(const Point& point, const T& val);

	friend Point operator*(const Point& point1, const Point& point2);
	template<class T>
	friend Point operator*(const Point& point, const T& val);
	template<class T>
	friend Point operator*(const T& val, const Point& point);

	friend Point operator/(const Point& point1, const Point& point2);
	template<class T>
	friend Point operator/(const Point& point, const T& val);
	template<class T>
	friend Point operator/(const T& val, const Point& point);

	friend Point operator+=(Point& point1, const Point& point2);
	template<class T>
	friend Point operator+=(Point& point, const T& val);

	friend Point operator-=(Point& point1, const Point& point2);
	template<class T>
	friend Point operator-=(Point& point, const T& val);

	friend Point operator*=(Point& point1, const Point& point2);
	template<class T>
	friend Point operator*=(Point& point, const T& val);

	friend Point operator/=(Point& point1, const Point& point2);
	template<class T>
	friend Point operator/=(Point& point, const T& val);

	friend bool operator==(const Point& point1, const Point& point2);
	friend bool operator!=(const Point& point1, const Point& point2);

	friend bool operator<(const Point& point1, const Point& point2);
	template<class T>
	friend bool operator<(const T& val, const Point& point1);
	template<class T>
	friend bool operator<(const Point& point1, const T& val);

	friend bool operator<=(const Point& point1, const Point& point2);
	template<class T>
	friend bool operator<=(const T& val, const Point& point1);
	template<class T>
	friend bool operator<=(const Point& point1, const T& val);

	friend bool operator>(const Point& point1, const Point& point2);
	template<class T>
	friend bool operator>(const T& val, const Point& point1);
	template<class T>
	friend bool operator>(const Point& point1, const T& val);

	friend bool operator>=(const Point& point1, const Point& point2);
	template<class T>
	friend bool operator>=(const T& val, const Point& point1);
	template<class T>
	friend bool operator>=(const Point& point1, const T& val);

	// template<class T>
	// friend bool operator<(const Point& point1, const T& anything_else){ return false; };
	// template<class T>
	// friend bool operator<(const T& anything_else, const Point& point1){ return false; };

	// template<class T>
	// friend bool operator<=(const Point& point1, const T& anything_else){ return false; };
	// template<class T>
	// friend bool operator<=(const T& anything_else, const Point& point1){ return false; };

	// template<class T>
	// friend bool operator>(const Point& point1, const T& anything_else){ return false; };
	// template<class T>
	// friend bool operator>(const T& anything_else, const Point& point1){ return false; };

	// template<class T>
	// friend bool operator>=(const Point& point1, const T& anything_else){ return false; };
	// template<class T>
	// friend bool operator>=(const T& anything_else, const Point& point1){ return false; };
	
	template<class T>
	friend bool operator==(const Point& point1, const T& anything_else){ return false; };
	template<class T>
	friend bool operator==(const T& anything_else, const Point& point1){ return false; };
	
	template<class T>
	friend bool operator!=(const Point& point1, const T& anything_else){ return true; };
	template<class T>
	friend bool operator!=(const T& anything_else, const Point& point1){ return true; };

	friend std::ostream& operator<<(std::ostream& os, const Point& point);

	bool            equal(Point other);
	std::vector<double>  get_as_polar();

	Point get_absolute();
	void calculate();
	void set_zero();
	void set_inf();
};

}