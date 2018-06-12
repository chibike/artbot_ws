#pragma once

#include "pi_pathfinder_header.h"

namespace blink
{

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

		//Point intersecting_point = Point(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
		/*
		* x and y should be the largest value with the same gradient 
		*/
		// Point intersecting_point = Point(99999*current_line.end.x, 99999*current_line.end.y);
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

		// cout << "------------------" << endl << endl << endl;

		if (!is_within_exclude_region)
		{
			shading_lines_buffer.push_back(shading_line);
		}
	}

	return shading_lines_buffer;
}

}