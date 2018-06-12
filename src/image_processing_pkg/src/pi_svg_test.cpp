

#include <vector>
#include <iostream>
#include "pi_color.h"
#include "pi_video.h"
#include "pi_svg_parser.h"
#include "opencv2/opencv.hpp"


void print_data(std::vector<std::string> data)
{
	std::cout << "------------------------------------------------" << std::endl;
	for (int i=0; i<data.size(); i++)
	{
		std::cout << data.at(i) << std::endl << std::endl;
	}
	std::cout << "------------------------------------------------" << std::endl;
}

void print_status(std::string status)
{
	std::cout << "status:: " << status << std::endl;
}

void test_1()
{
	std::cout << "***********************************************************************************************" << std::endl;
	print_status("starting...");

	print_status("reading file");
	std::string filename = "/home/ros/image_processing_ws/src/image_processing_pkg/svg/logo.svg";
	blink::XmlParser xml_parser(filename);

	print_status("fetching data: title");
	std::vector<std::string> data;
	if ( xml_parser.get_data(std::string("title"), data) )
	{
		print_status("printing...");
		print_data(data);
	}

	print_status("fetching params for: path");
	data.clear();
	if ( xml_parser.get_params(std::string("path"), data) )
	{
		print_status("printing...");
		print_data(data);
	}

	print_status("fetching params for: polygon");
	data.clear();
	if ( xml_parser.get_params(std::string("polygon"), data) )
	{
		print_status("printing...");
		print_data(data);
	}

	print_status("fetching params for: circle");
	data.clear();
	if ( xml_parser.get_params(std::string("circle"), data) )
	{
		print_status("printing...");
		print_data(data);
	}

	print_status("done!");
	std::cout << "***********************************************************************************************" << std::endl;
}

void test_2()
{
	std::cout << "***********************************************************************************************" << std::endl;
	print_status("starting...");

	print_status("reading file");
	std::string filename = "/home/ros/image_processing_ws/src/image_processing_pkg/svg/complex.svg";
	blink::SvgParser svg_parser;
	svg_parser.load_file(filename);

	print_status("parsing paths");
	svg_parser.extract_paths();

	print_status("done!");

	std::cout << "***********************************************************************************************" << std::endl;
}

void test_3()
{
	std::cout << "***********************************************************************************************" << std::endl;
	blink::Line line = blink::Line(blink::Point(0,0), blink::Point(0,5));
	std::cout << "Line = " << line << std::endl;
	std::cout << "get_length = " << line.get_length() << std::endl;
	std::cout << "measure_segment = " << measure_segment(line, line.start, line.end, 5, 0) << std::endl;

	line = blink::Line(blink::Point(0,0), blink::Point(5,5));
	std::cout << "get_length = " << line.get_length() << std::endl;
	std::cout << "measure_segment = " << measure_segment(line, line.start, line.end, 5, 0) << std::endl;

	line = blink::Line(blink::Point(0,0), blink::Point(0,-5));
	std::cout << "get_length = " << line.get_length() << std::endl;
	std::cout << "measure_segment = " << measure_segment(line, line.start, line.end, 5, 0) << std::endl;

	line = blink::Line(blink::Point(0,0), blink::Point(-5,-5));
	std::cout << "get_length = " << line.get_length() << std::endl;
	std::cout << "measure_segment = " << measure_segment(line, line.start, line.end, 5, 0) << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
}

void test_4()
{
	std::cout << "***********************************************************************************************" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	blink::CubicBezier cubic_bezier = blink::CubicBezier(blink::Point(0,0), blink::Point(0,5), blink::Point(-1.0,1), blink::Point(-1,4));
	std::cout << "CubicBezier = " << cubic_bezier << std::endl;
	std::cout << "cubic_bezier.get_length = " << cubic_bezier.get_length(5) << std::endl;

	blink::Line line = blink::Line(blink::Point(0,-5), blink::Point(0,0));
	std::cout << "cubic_bezier.is_smooth_from line = " << cubic_bezier.is_smooth_from(line) << std::endl;

	std::cout << "cubic_bezier.is_smooth_from self = " << cubic_bezier.is_smooth_from(cubic_bezier) << std::endl;

	std::cout << "---------------------------------------------------" << std::endl;
	for (float i=0; i<1.1; i+=0.1)
	{
		std::cout << "cubic_bezier.get_point(" << i << ") = " << cubic_bezier.get_point(i) << std::endl;
	}
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
}

void test_5()
{
	std::cout << "***********************************************************************************************" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	blink::QuadraticBezier quadratic_bezier = blink::QuadraticBezier(blink::Point(0,0), blink::Point(0,5), blink::Point(-1.0,1));
	std::cout << "quadratic_bezier = " << quadratic_bezier << std::endl;
	std::cout << "quadratic_bezier.get_length = " << quadratic_bezier.get_length(5) << std::endl;

	blink::Line line = blink::Line(blink::Point(0,-5), blink::Point(0,0));
	std::cout << "quadratic_bezier.is_smooth_from line = " << quadratic_bezier.is_smooth_from(line) << std::endl;

	std::cout << "quadratic_bezier.is_smooth_from self = " << quadratic_bezier.is_smooth_from(quadratic_bezier) << std::endl;

	std::cout << "---------------------------------------------------" << std::endl;
	for (float i=0; i<1.1; i+=0.1)
	{
		std::cout << "quadratic_bezier.get_point(" << i << ") = " << quadratic_bezier.get_point(i) << std::endl;
	}
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
}

void test_6()
{
	std::cout << "***********************************************************************************************" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	blink::Arc arc = blink::Arc(blink::Point(0,0), blink::Point(5,0), 2.5, 2.5, 0, true, true);
	std::cout << "arc = " << arc << std::endl;
	std::cout << "arc.get_length = " << arc.get_length(5) << std::endl;

	std::cout << "---------------------------------------------------" << std::endl;
	for (float i=0; i<1.1; i+=0.1)
	{
		std::cout << "arc.get_point(" << i << ") = " << arc.get_point(i) << std::endl;
	}
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "---------------------------------------------------" << std::endl;
	std::cout << "***********************************************************************************************" << std::endl;
}

void test_7(std::string filename)
{
	std::cout << "***************************************************************************************" << std::endl;
	print_status("starting...");

	print_status("reading file");
	blink::SvgParser svg_parser;
	svg_parser.load_file(filename);

	print_status("parsing paths");
	svg_parser.extract_paths();

	cv::Mat frame;
	if (!svg_parser.get_preview(frame))
	{
		print_status("could not preview the frame");
	}

	if (frame.empty())
	{
		std::cout << "error! the frame is empty" << std::endl;
	}
	else
	{
		std::cout << "starting viewer" << std::endl;

		// visualise
		blink::Viewer view(4, "svg_tests"); view.start();

		std::cout << "done" << std::endl;

		while (1)
		{
			int key = view.show_frame(frame);
			if ( key == 113 )
			{
				break;
			}
		}
		view.stop();
	}

	print_status("done!");

	std::cout << "***************************************************************************************" << std::endl;
}

int main(int argc, char const *argv[])
{
	// test_1();
	// test_2();
	// test_3();
	// test_4();
	// test_5();
	// test_6();

	for (int i=0; i<argc; i++)
	{
		std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
	}

	std::string filename = "/home/ros/image_processing_ws/src/image_processing_pkg/svg/logo1.svg";
	if (argc > 1)
	{
		filename = argv[1];
	}

	test_7(filename);
	//test_7("/home/ros/image_processing_ws/src/image_processing_pkg/svg/flower.svg");

	return 0;
}