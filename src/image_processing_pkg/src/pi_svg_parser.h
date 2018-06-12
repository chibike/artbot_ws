#pragma once

#include "pi_objects.h"
#include "pi_pathifier.h"
#include "pi_xml_parser.h"
#include "pi_sketch_sheet.h"

#include <vector>
#include <string>
#include <ctype.h>
#include <iostream>

#define MODE_DEBUG 0
#define MODE_RELEASE 1
#define SVG_PARSER_RUN_MODE MODE_RELEASE

namespace blink
{

class SvgParser
{
private:
	XmlParser _xml_parser;
	SketchSheet _sketch_sheet;
	double _resolution;

	bool _eval(std::string string_in, double &number_out);
	bool _get_next_value(std::string string_in, std::string &string_out, double &number_out, const std::string &seperator);
	bool _parse_svg_path(std::string &path_str);
	bool get_value(std::string &params, const std::string &param_name, double &out);
	bool get_value(std::string &params, const std::string &param_name, std::string &out);

	bool parse_line();
	bool parse_circle();
	bool parse_ellipse();
	bool parse_rect();
	bool parse_polygon();
	bool parse_path();

public:
	SvgParser();

	bool load_file(const std::string &filename);
	bool extract_paths();

	bool get_preview(cv::Mat &frame);
	bool get_paths(std::vector<Path> &paths);
	bool derieve_sheet_size(int top_padding, int bottom_padding, int left_padding, int right_padding, double scale);
};

SvgParser::SvgParser()
{
	_sketch_sheet.reserve(50);
	_resolution = 0.1;
}

bool SvgParser::load_file(const std::string &filename)
{
	return this->_xml_parser.load_file(filename);
}

bool SvgParser::derieve_sheet_size(int top_padding, int bottom_padding, int left_padding, int right_padding, double scale)
{
	return this->_sketch_sheet.derieve_sheet_size(top_padding, bottom_padding, left_padding, right_padding, scale);
}

bool SvgParser::get_preview(cv::Mat &frame)
{
	if (!this->_sketch_sheet.derieve_sheet_size())
	{
		return false;
	}

	int width = (int) std::round(this->_sketch_sheet.width + 1);
	int height = (int) std::round(this->_sketch_sheet.height + 1);

	#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "width: " << width << std::endl;
		std::cout << "height: " << height << std::endl;
	#endif

	cv::Scalar bg_color = cv::Scalar(0,0,0);
	cv::Scalar fg_color = cv::Scalar(255, 255, 255);

	return this->_sketch_sheet.get_sketches_as_cv_mat(frame, width, height, bg_color, fg_color);
}

bool SvgParser::get_paths(std::vector<Path> &paths_out)
{
	return this->_sketch_sheet.get_sketches_as_paths(paths_out);
}

bool SvgParser::extract_paths()
{
	if ( !parse_circle() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing circles not successful" << std::endl;
		#endif
	}

	if ( !parse_ellipse() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing ellipse not successful" << std::endl;
		#endif
	}

	if ( !parse_rect() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing rect not successful" << std::endl;
		#endif
	}

	if ( !parse_line() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing lines not successful" << std::endl;
		#endif
	}

	if ( !parse_polygon() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing polygons not successful" << std::endl;
		#endif
	}

	if ( !parse_path() )
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "parsing paths not successful" << std::endl;
		#endif
	}

	return true;
}

bool SvgParser::get_value(std::string &params, const std::string &param_name, std::string &out)
{
	std::size_t start = 0, end = 0;

	// find the parameter
	start = params.find(param_name , start);
	if (start == string::npos)
		return false;

	// find the starting quote
	start = params.find("\"", start);
	if (start == string::npos)
		return false;
	start++;

	// end at the ending quote
	end = params.find("\"", start);
	if (end == string::npos)
		return false;

	out = params.substr(start, end-start);
	return true;
}

bool SvgParser::get_value(std::string &params, const std::string &param_name, double &out)
{
	std::string value;

	if ( !this->get_value(params, param_name, value) )
		return false;

	// ADD:: try catch for stod
	out = std::stod(value);
	return true;
}

bool SvgParser::parse_circle()
{
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("circle"), data))
		return false;

	double cx, cy, r;
	std::string params;
	for (int i=0; i<data.size(); i++)
	{
		params = data.at(i);

		if ( !get_value(params, std::string("cx"), cx) )
			return false;

		if ( !get_value(params, std::string("cy"), cy) )
			return false;

		if ( !get_value(params, std::string("r"), r) )
			return false;

		if (!_sketch_sheet.add_circle(Point(cx, cy), r, 2.0*M_PI, 32))
		{
			std::cout << "_sketch_sheet add_circle failed" << std::endl;
			return false;
		}
	}

	return true;
}

bool SvgParser::parse_ellipse()
{
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("ellipse"), data))
		return false;

	std::string params;
	double cx, cy, rx, ry;
	for (int i=0; i<data.size(); i++)
	{
		params = data.at(i);

		if ( !get_value(params, std::string("cx"), cx) )
			return false;

		if ( !get_value(params, std::string("cy"), cy) )
			return false;

		if ( !get_value(params, std::string("rx"), rx) )
			return false;

		if ( !get_value(params, std::string("ry"), ry) )
			return false;

		if (!_sketch_sheet.add_ellipse(Point(cx, cy), rx, ry, 2.0*M_PI, 32))
		{
			std::cout << "_sketch_sheet add_ellipse failed" << std::endl;
			return false;
		}
	}

	return true;
}

bool SvgParser::parse_rect()
{
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("rect"), data))
	{
		#if SVG_PARSER_RUN_MODE == MODE_DEBUG
		std::cout << "could not get params" << std::endl;
		#endif

		return false;
	}

	std::string params;
	bool successful = true, has_rx = false, has_ry = false;
	double x, y, width, height, rx, ry;
	for (int i=0; i<data.size(); i++)
	{
		params = data.at(i);
		has_rx = true, has_ry = true;

		if ( !get_value(params, std::string("x"), x) )
		{
			successful = false;
			continue;
		}

		if ( !get_value(params, std::string("y"), y) )
		{
			successful = false;
			continue;
		}

		if ( !get_value(params, std::string("width"), width) )
		{
			successful = false;
			continue;
		}

		if ( !get_value(params, std::string("height"), height) )
		{
			successful = false;
			continue;
		}

		if ( !get_value(params, std::string("rx"), rx) )
		{
			has_rx = false;
		}

		if ( !get_value(params, std::string("ry"), ry) )
		{
			has_ry = false;
		}

		if (!_sketch_sheet.add_rect(Point(x,y), width, height))
		{
			std::cout << "_sketch_sheet add_rect failed" << std::endl;
			return false;
		}
		
		if (has_ry)
		{
			// std::cout << "ry : "     << ry     << ";" << std::endl;
		}
		if (has_rx)
		{
			// std::cout << "rx : "     << rx     << ";" << std::endl;
		}
	}

	return successful;
}

bool SvgParser::parse_line()
{
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("line"), data))
		return false;

	double x1, x2, y1, y2;
	std::string params;
	for (int i=0; i<data.size(); i++)
	{
		params = data.at(i);

		if ( !get_value(params, std::string("x1"), x1) )
			return false;

		if ( !get_value(params, std::string("x2"), x2) )
			return false;

		if ( !get_value(params, std::string("y1"), y1) )
			return false;

		if ( !get_value(params, std::string("y2"), y2) )
			return false;

		if (!_sketch_sheet.add_line(Point(x1,y1), Point(x2,y2)))
		{
			std::cout << "_sketch_sheet add_line failed" << std::endl;
			return false;
		}
	}

	return true;
}

bool SvgParser::parse_path()
{
	// get params as string from polygon
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("path"), data))
	{
		return false;
	}

	for (int i=0; i<data.size(); i++)
	{
		// get points value as string from polygon params
		std::string d_str;
		if ( !get_value(data.at(i), std::string("d"), d_str) )
		{
			return false;
		}

		std::string sub_part;
		bool finished = false;
		std::size_t start=0, end=0;

		while(1)
		{
			// end at the ending quote
			end = d_str.find(" ", start);
			if (end == string::npos)
			{
				finished = true;
			}

			sub_part = d_str.substr(start, end-start);
			start = end + 1;

			this->_parse_svg_path(sub_part);

			if (finished)
			{
				break;
			}
		}
	}

	return true;
}

bool SvgParser::parse_polygon()
{
	// get params as string from polygon
	std::vector<std::string > data;
	if (!_xml_parser.get_params(std::string("polygon"), data))
	{
		return false;
	}

	for (int i=0; i<data.size(); i++)
	{
		// get points value as string from polygon params
		std::string points_str;
		if ( !get_value(data.at(i), std::string("points"), points_str) )
		{
			return false;
		}

		// may need to strip whitespace from points_str

		std::string str_part;
		bool finished = false;
		std::vector<double> points_buffer;
		std::size_t start = 0, end = 0, comma=0;

		// convert points_str to a vector of points
		while (1)
		{
			// end at the ending quote
			end = points_str.find(" ", start);
			if (end == string::npos)
			{
				finished = true;
			}

			str_part = points_str.substr(start, end-start);

			comma = points_str.find(",", start);
			if (comma == string::npos)
			{
				// ADD:: try catch for stod
				points_buffer.emplace_back(std::stod(str_part));
			}
			else
			{
				points_buffer.emplace_back(std::stod(points_str.substr(start, comma-start)));
				points_buffer.emplace_back(std::stod(points_str.substr(comma+1, end-start)));
			}

			start = end + 1;

			if (finished)
			{
				break;
			}
		}

		std::vector<Point> points;
		points.reserve(points_buffer.size() + 1);

		int index;
		double x,y;

		// converts vector of points to individual vectors of x and y points
		for (int i=0; i<points_buffer.size()/2; i++)
		{
			index = i * 2;
			x = points_buffer.at(index);
			y = points_buffer.at(index+1);

			points.emplace_back(Point(x,y));

			// std::cout << "x= " << x << ",  "  << "y= "  << y << ";" << std::endl;
		}

		if (!_sketch_sheet.add_path(points, false))
		{
			std::cout << "_sketch_sheet add_path failed" << std::endl;
			return false;
		}
	}

	return true;
}

bool SvgParser::_eval(std::string string_in, double &number_out)
{
	std::string::size_type sz;
	number_out = std::stod(string_in, &sz);
	string_in = string_in.substr(sz);

	std::string operator_str;
	while (string_in.length() > 0)
	{
		operator_str = string_in.substr(0, 1);
		if (operator_str.compare(" ") == 0)
		{
			string_in = string_in.substr(1);
			continue;
		}
		else if (operator_str.compare("-") == 0)
		{
			number_out -= std::stod(string_in.substr(1), &sz);
			string_in = string_in.substr(++sz);
			continue;
		}
		else if (operator_str.compare("+") == 0)
		{
			number_out += std::stod(string_in.substr(1), &sz);
			string_in = string_in.substr(++sz);
			continue;
		}
		else
		{
			std::cout << "error:: could not identify operator: " << operator_str << std::endl;
		}
		
		string_in = string_in.substr(1);
	}


	return true;
}


bool SvgParser::_get_next_value(std::string string_in, std::string &string_out, double &number_out, const std::string &seperator)
{
	std::string sub_part;
	std::string character;
	unsigned int start = 0, end = 0;

	// 1. remove all leading spaces or separating characters
	character = string_in.substr(start, 1);
	while ((string_in.length() > 0) && (character.compare(seperator) == 0 || character.compare(" ") == 0))
	{
		string_in = string_in.substr(1);
		character = string_in.substr(start, 1);
	}

	// 2. find the end of the value in the string
	int dot_count = 0;
	unsigned int index;
	for (index=start; index<string_in.length(); index++)
	{
		character = string_in.substr(index, 1);
		if (isdigit(character.c_str()[0]))
		{
			continue;
		}
		else if(character.compare(".") == 0 && (0 >= dot_count++))
		{
			continue;
		}
		else if (character.compare("-") == 0 && index == start)
		{}
		else
		{
			break;
		}
	}

	end = index;

	sub_part = string_in.substr(start, end-start);
	string_out = string_in.substr(end);

	// std::cout << "_get_next_value:: sub_part: " << sub_part << std::endl;
	// std::cout << "_get_next_value:: string_out: " << string_out << std::endl;

	if ( !this->_eval(sub_part, number_out) )
	{
		std::cout << "#f_get_next_value:: number_out: " << number_out << std::endl;
		return false;
	}

	// std::cout << "#t_get_next_value:: number_out: " << number_out << std::endl;

	return true;
}

bool SvgParser::_parse_svg_path(std::string &d_str)
{
	unsigned int current_pos, end;
	std::string cmd;
	std::string last_cmd;

	if (!_sketch_sheet.start_sketch(50))
	{
		std::cout << "_sketch_sheet! could not start sketch" << std::endl;
		return false;
	}

	while (d_str.length() > 0)
	{
		current_pos = 0;
		end = current_pos + 1;
		cmd = d_str.substr(current_pos, end-current_pos);

		if (cmd.compare("-") != 0 && cmd.compare(".") != 0)
		{
			current_pos = end;
			d_str = d_str.substr(current_pos);
		}

		if (cmd.compare(",") == 0 || cmd.compare(" ") == 0 || cmd.compare("-") == 0 || cmd.compare(".") == 0)
		{
			cmd = last_cmd;
		}

		if (cmd.compare("M") == 0)
		{
			double x, y;
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.moveto(Point(x,y)) )
			{
				std::cout << "error! could not move to point" << std::endl;
			}
		}
		else if (cmd.compare("L") == 0)
		{
			double x, y;
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.lineto(Point(x,y)) )
			{
				std::cout << "error! could not line to point" << std::endl;
			}
		}
		else if (cmd.compare("V") == 0)
		{
			double value;
			this->_get_next_value(d_str, d_str, value, std::string(","));

			if ( !_sketch_sheet.vertical_lineto(value) )
			{
				std::cout << "error! could not vertical to y value" << std::endl;
			}
		}
		else if (cmd.compare("H") == 0)
		{
			double value;
			this->_get_next_value(d_str, d_str, value, std::string(","));

			if ( !_sketch_sheet.horizontal_lineto(value) )
			{
				std::cout << "error! could not horizontal to x value" << std::endl;
			}
		}
		else if (cmd.compare("Q") == 0)
		{
			double cx,cy,x,y;

			this->_get_next_value(d_str, d_str, cx, std::string(","));
			this->_get_next_value(d_str, d_str, cy, std::string(","));
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.quadratic_curveto(Point(x,y), Point(cx,cy), _resolution) )
			{
				std::cout << "error! could not quadratic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("T") == 0)
		{
			double x,y;

			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.quadratic_curveto(Point(x,y), _resolution) )
			{
				std::cout << "error! could not short quadratic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("C") == 0)
		{
			double c1x,c1y,c2x,c2y,x,y;

			this->_get_next_value(d_str, d_str, c1x, std::string(","));
			this->_get_next_value(d_str, d_str, c1y, std::string(","));
			this->_get_next_value(d_str, d_str, c2x, std::string(","));
			this->_get_next_value(d_str, d_str, c2y, std::string(","));
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.cubic_curveto(Point(x,y), Point(c1x, c1y), Point(c2x, c2y), _resolution) )
			{
				std::cout << "error! could not cubic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("S") == 0)
		{
			double c2x,c2y,x,y;

			this->_get_next_value(d_str, d_str, c2x, std::string(","));
			this->_get_next_value(d_str, d_str, c2y, std::string(","));
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			if ( !_sketch_sheet.cubic_curveto(Point(x,y), Point(c2x, c2y), _resolution) )
			{
				std::cout << "error! could not short cubic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("A") == 0)
		{
			double rx,ry,x_axis_rotate,large_arc_flag,sweep_flag,x,y;

			this->_get_next_value(d_str, d_str, rx, std::string(","));
			this->_get_next_value(d_str, d_str, ry, std::string(","));
			this->_get_next_value(d_str, d_str, x_axis_rotate, std::string(","));
			this->_get_next_value(d_str, d_str, large_arc_flag, std::string(","));
			this->_get_next_value(d_str, d_str, sweep_flag, std::string(","));
			this->_get_next_value(d_str, d_str, x, std::string(","));
			this->_get_next_value(d_str, d_str, y, std::string(","));

			bool arc   = std::round(large_arc_flag) != 0;
			bool sweep = std::round(sweep_flag)     != 0;

			if ( !_sketch_sheet.arcto(Point(x,y), rx, ry, x_axis_rotate, arc, sweep, _resolution) )
			{
				std::cout << "error! could not arc to position" << std::endl;
			}
		}
		else if (cmd.compare("Z") == 0)
		{
			if (!_sketch_sheet.stop_sketch(false))
			{
				std::cout << "_sketch_sheet! could not stop sketch" << std::endl;
				return false;
			}

			if (!_sketch_sheet.start_sketch(50))
			{
				std::cout << "_sketch_sheet! could not start sketch" << std::endl;
				return false;
			}
		}
		else if (cmd.compare("m") == 0)
		{
			double dx, dy;
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.moveto(Point(dx, dy), true) )
			{
				std::cout << "error! could not move to point" << std::endl;
			}
		}
		else if (cmd.compare("l") == 0)
		{
			double dx, dy;
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.lineto(Point(dx, dy), true) )
			{
				std::cout << "error! could not line to point" << std::endl;
			}
		}
		else if (cmd.compare("v") == 0)
		{
			double dvalue;
			this->_get_next_value(d_str, d_str, dvalue, std::string(","));

			if ( !_sketch_sheet.vertical_lineto(dvalue, true) )
			{
				std::cout << "error! could not vertical to y value" << std::endl;
			}
		}
		else if (cmd.compare("h") == 0)
		{
			double dvalue;
			this->_get_next_value(d_str, d_str, dvalue, std::string(","));

			if ( !_sketch_sheet.horizontal_lineto(dvalue, true) )
			{
				std::cout << "error! could not horizontal to x value" << std::endl;
			}
		}
		else if (cmd.compare("q") == 0)
		{
			double dcx,dcy,dx,dy;
			this->_get_next_value(d_str, d_str, dcx, std::string(","));
			this->_get_next_value(d_str, d_str, dcy, std::string(","));
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.quadratic_curveto(Point(dx, dy), Point(dcx, dcy), _resolution, true) )
			{
				std::cout << "error! could not quadratic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("t") == 0)
		{
			double dx,dy;
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.quadratic_curveto(Point(dx, dy), _resolution, true))
			{
				std::cout << "error! could not quadratic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("c") == 0)
		{
			double dc1x,dc1y,dc2x,dc2y,dx,dy;
			this->_get_next_value(d_str, d_str, dc1x, std::string(","));
			this->_get_next_value(d_str, d_str, dc1y, std::string(","));
			this->_get_next_value(d_str, d_str, dc2x, std::string(","));
			this->_get_next_value(d_str, d_str, dc2y, std::string(","));
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.cubic_curveto(Point(dx, dy), Point(dc1x, dc1y), Point(dc2x, dc2y), _resolution, true) )
			{
				std::cout << "error! could not cubic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("s") == 0)
		{
			double dc2x,dc2y,dx,dy;
			this->_get_next_value(d_str, d_str, dc2x, std::string(","));
			this->_get_next_value(d_str, d_str, dc2y, std::string(","));
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			if ( !_sketch_sheet.cubic_curveto(Point(dx, dy), Point(dc2x, dc2y), _resolution, true) )
			{
				std::cout << "error! could not cubic_curve to position" << std::endl;
			}
		}
		else if (cmd.compare("a") == 0)
		{
			double drx,dry,x_axis_rotate,large_arc_flag,sweep_flag,dx,dy;

			this->_get_next_value(d_str, d_str, drx, std::string(","));
			this->_get_next_value(d_str, d_str, dry, std::string(","));
			this->_get_next_value(d_str, d_str, x_axis_rotate, std::string(","));
			this->_get_next_value(d_str, d_str, large_arc_flag, std::string(","));
			this->_get_next_value(d_str, d_str, sweep_flag, std::string(","));
			this->_get_next_value(d_str, d_str, dx, std::string(","));
			this->_get_next_value(d_str, d_str, dy, std::string(","));

			bool arc   = std::round(large_arc_flag) != 0;
			bool sweep = std::round(sweep_flag)     != 0;

			if ( !_sketch_sheet.arcto(Point(dx, dy), drx, dry, x_axis_rotate, arc, sweep, _resolution, true) )
			{
				std::cout << "error! could not arc to position" << std::endl;
			}
		}
		else if (cmd.compare("z") == 0)
		{
			if (!_sketch_sheet.stop_sketch(false))
			{
				std::cout << "_sketch_sheet! could not stop sketch" << std::endl;
				return false;
			}

			if (!_sketch_sheet.start_sketch(50))
			{
				std::cout << "_sketch_sheet! could not start sketch" << std::endl;
				return false;
			}
		}
		else
		{
			std::cout << "invalid command \"" << cmd << "\"" << std::endl;
			break;
		}

		last_cmd = cmd;
	}

	if (!_sketch_sheet.stop_sketch(false))
	{
		std::cout << "_sketch_sheet! could not stop sketch" << std::endl;
		return false;
	}

	return true;
}

}