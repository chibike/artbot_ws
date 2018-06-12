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

}