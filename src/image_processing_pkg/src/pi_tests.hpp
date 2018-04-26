#ifndef BLINK_PI_TESTS_HPP
#define BLINK_PI_TESTS_HPP

#include "pi_color.h"
#include "pi_video.h"
#include "pi_objects.h"
#include "pi_color_space.h"
#include "opencv2/opencv.hpp"

#include <iostream>
using namespace std;

//#define USE_CAMERA

cv::RNG rng(12345);

namespace blink
{
	const string test_image_name = "/home/chibike/catkin_ws/src/mdx_testbot_com/images/img_001.jpeg";

	void test_video()
	{
		Viewer view; view.start();
		#ifdef USE_CAMERA
			Camera cam(1);
			cam.start();
		#else
			StaticImage still(test_image_name);
			still.start();
		#endif

		while (1)
		{
			#ifdef USE_CAMERA
				cv::Mat frame = cam.get_next_frame();
			#else
				cv::Mat frame = still.get_next_frame();
			#endif

			int key = view.show_frame(frame);
			if ( key == 113 || key < 0 )
			{
				break;
			}
		}

		#ifdef USE_CAMERA
			cam.stop();
		#else
			still.stop();
		#endif
		view.stop();
		
	}

	void test_simplify_image()
	{
		Viewer view; view.start();
		#ifdef USE_CAMERA
			Camera cam(1);
			cam.start();
		#else
			StaticImage still(test_image_name);
			still.start();
		#endif

		std::vector<RGBColor> color_array;
		
		color_array.push_back(BLACK_MAX);
		color_array.push_back(WHITE_MAX);
		color_array.push_back(RED_MAX);
		color_array.push_back(BLUE_MAX);
		color_array.push_back(GREEN_MAX);
		color_array.push_back(YELLOW_MAX);
		color_array.push_back(MAGENTA_MAX);
		color_array.push_back(CYAN_MAX);

		cv::Mat frame;

		while (1)
		{
			#ifdef USE_CAMERA
				cv::Mat frame = cam.get_next_frame();
			#else
				cv::Mat frame = still.get_next_frame();
			#endif

			simplify_image(frame, color_array);
			int key = view.show_frame(frame);
			if ( key == 113 || key < 0 )
			{
				break;
			}
		}

		#ifdef USE_CAMERA
			cam.stop();
		#else
			still.stop();
		#endif
		view.stop();
	}

	void generate_path_from_image()
	{
		const string test_image_name = "/home/chibike/catkin_ws/src/mdx_testbot_com/images/test_portrait.jpg";
		const string save_image_name_final = "/home/chibike/catkin_ws/src/mdx_testbot_com/images/test_portrait_save_final.jpg";

		StaticImage still(test_image_name);
		still.start();

		cv::Mat frame = still.get_next_frame();
		cv::resize(frame, frame, cv::Size(640,480), 0, 0, CV_INTER_AREA);

		std::cout << "-- Image Stats --" << std::endl;
		std::cout << "width: " << frame.cols << std::endl;
		std::cout << "height: " << frame.rows << std::endl << std::endl;

		// --------- 

		// std::cout << "-- New Color Space Stats --" << std::endl;
		// std::cout << "depth: " << 5 << std::endl;
		// std::cout << "number of colors: " << color_space_5d::color_array.size() << std::endl << std::endl;

		// std::cout << "simplifying image ...." << std::endl;
		// simplify_image(frame, color_space_5d::color_array);
		// std::cout << "Done!" << std::endl << std::endl;

		// --------- 

		// --------- 

		std::cout << "-- New Color Space Stats --" << std::endl;
		std::cout << "depth: " << 3 << std::endl;
		std::cout << "number of colors: " << color_space_3d::color_array.size() << std::endl << std::endl;

		std::cout << "simplifying image ...." << std::endl;
		simplify_image(frame, color_space_3d::color_array);
		std::cout << "Done!" << std::endl << std::endl;

		// --------- 

		std::cout << "applying blur..." << std::endl;
		cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0, 0);
		std::cout << "Done!" << std::endl << std::endl;

		cv::Mat edge_frame, view_frame;
		std::cout << "Getting the edges" << std::endl;
		cv::Canny(frame, edge_frame, 50, 150, 3);
		std::cout << "Done!" << std::endl << std::endl;

		std::cout << "finding contours..." << std::endl;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(edge_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
		std::cout << "Done!" << std::endl << std::endl;

		std::cout << "drawing contours..." << std::endl;
		view_frame = cv::Mat::zeros(edge_frame.size(), CV_8UC3);
		for (int i=0; i<contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
			cv::drawContours(view_frame, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
		}

		cv::imwrite(save_image_name_final, view_frame);
		std::cout << "Done!" << std::endl << std::endl;

		std::cout << "converting contours to paths..." << std::endl;
		std::vector<Path> paths;
		for (int i=0; i<contours.size(); i++)
		{
			std::vector<cv::Point> contour = contours.at(i);
			Path path = Path(contour, true);
			paths.push_back(path);
		}
		std::cout << "Done!" << std::endl << std::endl;

		int before_size = paths.size();
		
		std::cout << "filtering paths..." << std::endl;
		for (int i=paths.size()-1; i>=0; i--)
		{
			RectInfo info = paths.at(i).get_rect_info();

			double factor_a = (info.perimeter/info.height);
			double factor_b = (info.perimeter/info.width);
			double factor_c = factor_a < factor_b ? factor_a : factor_b;

			if (factor_c < 7 && info.area < 200)
			{
				paths.erase(paths.begin() + i);
			}
		}
		std::cout << "Done!" << std::endl << std::endl;

		int after_size = paths.size();

		std::cout << "-- Paths Stats --" << std::endl;
		std::cout << "before number of paths :" << before_size << std::endl;
		std::cout << "after number of paths :" << after_size << std::endl << std::endl;

		std::cout << "Drawing after paths..." << std::endl;
		cv::Mat view_2_frame = cv::Mat::zeros(edge_frame.size(), CV_8UC3);
		for (int i=0; i<paths.size(); i++)
		{
			Path path = paths.at(i);
			cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

			path.draw(view_2_frame, color);
		}
		std::cout << "Done!" << std::endl << std::endl;		

		Viewer view_1 = Viewer(30, "before");
		Viewer view_2 = Viewer(30, "after");

		view_1.start();
		view_2.start();

		int key = 0;
		while (1)
		{
			key = view_1.show_frame(view_frame);
			if ( key == 113 )
			{
				view_1.stop();
				break;
			}

			key = view_2.show_frame(view_2_frame);
			if ( key == 113 )
			{
				view_2.stop();
				break;
			}
		}

		still.stop();
		view_1.stop();
		view_2.stop();
	}

	void test_objects()
	{
		cv::Mat frame;
		cv::Scalar path_color(255,0,0);
		cv::Scalar shade_color(0,255,0);

		std::vector<Point> points;
		std::vector<Line> lines;

		points.push_back(Point(240, 80));
		points.push_back(Point(400, 400));
		points.push_back(Point(80,  400));

		Path my_path = Path(points, false);
		Point centroid = my_path.calculate_centroid();
		Point center = my_path.rect_info.center;

		std::cout << "center: " << center.x << ", " << center.y << std::endl;
		std::cout << "centroid: " << centroid.x << ", " << centroid.y << std::endl;

		// visualise
		Viewer view(4, "objects"); view.start();
		while (1)
		{
			frame = cv::Mat(480, 480, CV_8UC3, cv::Scalar(0,0,0));

			// Line line = Line(Point(200, 240), Point(280, 240));
			// line.rotate(0.1);
			// line.draw(frame, shade_color);

			// line = Line(Point(200, 280), Point(280, 280));
			// line.rotate(0.1);
			// line.scale(2);
			// line.draw(frame, shade_color);

			// my_path.rotate(0.1);
			my_path.draw(frame, path_color);

			lines = my_path.get_intersecting_lines(10, 0.0);
			for (int i=0; i<lines.size(); i++)
			{
				Line line = lines.at(i);
				line.draw(frame, shade_color);
			}

			// lines = my_path.get_intersecting_lines(10, M_PI_2);
			// for (int i=0; i<lines.size(); i++)
			// {
			// 	Line line = lines.at(i);
			// 	line.draw(frame, shade_color);
			// }

			int key = view.show_frame(frame);
			if ( key == 113 )
			{
				break;
			}
		}
		view.stop();
	}

	void test_pathfinder_a()
	{
		cv::Mat frame;
		cv::Scalar red_color(0,0,255);
		cv::Scalar blue_color(255,0,0);
		cv::Scalar green_color(0,255,0);

		std::vector<Point> points;
		std::vector<Line> lines;

		// #Test Case #01
		points.push_back(Point(240, 80));
		points.push_back(Point(400, 400));
		points.push_back(Point(80,  400));

		Path path_1 = Path(points, false);

		points.clear();
		points.push_back(Point(280, 80));
		points.push_back(Point(440, 400));
		points.push_back(Point(120,  400));

		Path path_2 = Path(points, false);
		path_2.rotate(0.1);

		Point p1 = Point(240, 116);
		Point p2 = Point(295, 82);
		Line line = Line(p1, p2);

		if (path_1.is_touching(p1))
		{
			cout << "P1 is touching path_1" << endl;
		}
		if (path_1.is_touching(p2))
		{
			cout << "P2 is touching path_1" << endl;
		}

		if (path_2.is_touching(p1))
		{
			cout << "P1 is touching path_2" << endl;
		}
		if (path_2.is_touching(p2))
		{
			cout << "P2 is touching path_2" << endl;
		}

		Viewer view(1, "objects"); view.start();
		while (1)
		{
			frame = cv::Mat(480, 480, CV_8UC3, cv::Scalar(0,0,0));

			path_1.draw(frame, blue_color);
			path_2.draw(frame, green_color);

			line.draw(frame, red_color);

			int key = view.show_frame(frame);
			if ( key == 113 || key < 0 )
			{
				break;
			}
		}
		view.stop();
	}

	void test_pathfinder()
	{
		cv::Mat frame;
		cv::Scalar red_color(0,0,255);
		cv::Scalar blue_color(255,0,0);
		cv::Scalar green_color(0,255,0);

		std::vector<Point> points;
		std::vector<Line> lines;

		// #Test Case #01
		points.push_back(Point(240, 80));
		points.push_back(Point(400, 400));
		points.push_back(Point(80,  400));

		Path path_1 = Path(points, false);

		points.clear();
		points.push_back(Point(280, 80));
		points.push_back(Point(440, 400));
		points.push_back(Point(120,  400));

		Path path_2 = Path(points, false);
		path_2.rotate(0.1);

		// #Test Case #02
		// points.push_back(Point(240, 80));
		// points.push_back(Point(400, 400));
		// points.push_back(Point(80,  400));
		// points.push_back(Point(5,  200));

		// Path path_1 = Path(points, false);

		// points.clear();
		// points.push_back(Point(280, 80));
		// points.push_back(Point(440, 400));
		// points.push_back(Point(120,  400));

		// Path path_2 = Path(points, false);
		// path_2.rotate(-0.1);

		Pathfinder pathfinder;
		// Path path_3 = pathfinder.unite(path_1, path_2);
		Path path_3 = pathfinder.subtract(path_1, path_2);

		std::vector<Path> exclude_regions;
		exclude_regions.push_back(path_1);
		exclude_regions.push_back(path_2);
		exclude_regions.push_back(path_3);

		std::vector<Line> shade_lines = pathfinder.shade_frame(20, 20, 440, 440, 30, 0., exclude_regions);
		cout << "num of shade lines = " << shade_lines.size() << endl;

		Viewer view(1, "objects"); view.start();
		while (1)
		{
			frame = cv::Mat(480, 480, CV_8UC3, cv::Scalar(0,0,0));

			path_1.draw(frame, blue_color);
			path_2.draw(frame, green_color);
			path_3.draw(frame, red_color);

			for (int i=0; i<shade_lines.size(); i++)
			{
				Line line = shade_lines.at(i);
				line.draw(frame, red_color);
			}

			int key = view.show_frame(frame);
			if ( key == 113 || key < 0 )
			{
				break;
			}
		}
		view.stop();
	}
}

#endif /*BLINK_PI_TESTS_HPP*/