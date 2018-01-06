#include "image_processing_pkg/ProcessedImage.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

#include "opencv2/opencv.hpp"

#include "std_msgs/String.h"
#include <algorithm>
#include "ros/ros.h"
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <string>

cv::RNG rng(12345);

template <class T> const T& constrainf(const T& x, const T& min_x, const T& max_x)
{
	return std::min(max_x, std::max(min_x, x));
}

class ImageCapture
{
	private:
		int ratio;
		int counter;
		int kernel_size;
		int low_threshold;
		int edge_threshold;
		int max_low_threshold;
		
		cv::Mat __rgb_frame;
		cv::Mat __gray_frame;
		
		std::string __home_path;
		std::string __frame_name;
		std::vector<cv::Rect> __faces;
		std::vector<cv::Vec4i> __hierarchy;
		std::vector<cv::Rect> __focus_rects;
		std::vector<std::vector<cv::Point> > __contours;
		
		cv::CascadeClassifier __face_cascade;
		
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		//image_processing_pkg::ProcessedImage __processed_image;
		sensor_msgs::Image __processed_image;
		ros::Publisher __processed_image_pub;
		
		cv::Scalar __face_highlight_color;
	    cv::Scalar __body_highlight_color;

	    cv::VideoCapture camera;
		
	public:
		ImageCapture() : it_(nh_)
		{
		}
		
		~ImageCapture()
		{
			cv::destroyWindow(__frame_name);
		}
		void run();
		void stop();
		bool start();
		bool update();
		void run_once();
		void start_window();
		void process_image();
		void update_window();
		void update_low_threshold(int);
		void draw_contours(cv::Mat &frame);
		std::vector<cv::Rect> detect_faces();
		void highlight_persons(cv::Scalar, cv::Scalar);
};

bool ImageCapture::start()
{
	counter = 0;
	edge_threshold = 1;
	max_low_threshold = 100;
	ratio = 2;
	kernel_size = 3;
	__home_path = "/home/odroid/artbot_ws/src/image_processing_pkg";

	update_low_threshold(100);

	__face_highlight_color = cv::Scalar(0, 0, 255);
	__body_highlight_color = cv::Scalar(255, 0, 0);
	
	__face_cascade.load(__home_path + "/parameters/haarcascades/haarcascade_frontalface_default.xml");

	__frame_name = "edges";
	
	__rgb_frame = cv::imread(__home_path + "/images/box.png", CV_LOAD_IMAGE_COLOR);
	if (!__rgb_frame.data)
	{
		std::cout << "Could not read image" << std::endl;
		return false;
	}

	//__processed_image_pub = nh_.advertise<image_processing_pkg::ProcessedImage>("/processed_image", 1);
	__processed_image_pub = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
	
	camera = *(new cv::VideoCapture(0));
	if (!camera.isOpened())
	{
		return -1;
	}

	return true;
}

void ImageCapture::start_window()
{
	cv::namedWindow(__frame_name, cv::WINDOW_AUTOSIZE);
}

void ImageCapture::stop()
{
	//cv::destroyWindow(__frame_name);
}

bool ImageCapture::update()
{
	detect_faces();
	
	if (__faces.size() > 0)
	{
		process_image();
	}
	
	draw_contours(__rgb_frame);
	
	return true;
}

void ImageCapture::update_window()
{
	cv::imshow(__frame_name, __rgb_frame);
	cv::waitKey(10);
}

void ImageCapture::run()
{
	cv::Scalar face_highlight_color(0, 0, 255);
	cv::Scalar body_highlight_color(255, 0, 0);
	while (1)
	{
		run_once();
		update_window();
	}
}

void ImageCapture::run_once()
{
	try
	{
		camera >> __rgb_frame;
		
		update();
		highlight_persons(__face_highlight_color, __body_highlight_color);

		cv_bridge::CvImage out_msg;
		out_msg.header.seq = counter++;
		out_msg.header.stamp = ros::Time::now();
		out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
		out_msg.image = __rgb_frame;

		sensor_msgs::ImagePtr im_msg = out_msg.toImageMsg();
		__processed_image = *im_msg;

		//__processed_image.processed_image = *im_msg;
		//__processed_image.number_of_faces = __faces.size();
		__processed_image_pub.publish(__processed_image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (cv::Exception& e)
	{
		ROS_ERROR("cv exception: %s", e.what());
		std::cout << "cv exception: " << e.what() << std::endl;
	}
}

void ImageCapture::process_image()
{	
	cv::cvtColor(__rgb_frame, __gray_frame, cv::COLOR_BGR2GRAY);
	cv::blur(__gray_frame, __gray_frame, cv::Size(3, 3));
	
	cv::Mat canny_output;
	
	__contours.clear();	
	__hierarchy.clear();
	
	cv::Rect focus_rect ;
	std::vector<cv::Vec4i> hierarchy;
	std::vector<std::vector<cv::Point> > contours;
	for (int i=0; i<__focus_rects.size(); i++)
	{
		focus_rect = __focus_rects[i];
		
		cv::Mat gray_matrix = *(new cv::Mat(__gray_frame, focus_rect));
		cv::Canny(gray_matrix, canny_output, low_threshold, low_threshold*ratio, kernel_size);
		cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		
		for (int i=0; i<contours.size(); i++)
		{
			std::vector<cv::Point> contour = contours[i];
			for (int j=0; j<contour.size(); j++)
			{
				contour[j].x += focus_rect.x;
				contour[j].y += focus_rect.y;
			}
			__contours.push_back(contour);
		}
		
		for (int i=0; i<hierarchy.size(); i++)
		{
			__hierarchy.push_back(hierarchy[i]);
		}
	}
}

void ImageCapture::update_low_threshold(int track_position)
{
	low_threshold = track_position;
}

std::vector<cv::Rect> ImageCapture::detect_faces()
{
	__face_cascade.detectMultiScale(__rgb_frame, __faces, 1.3, 5);
	return __faces;
}

void ImageCapture::highlight_persons(cv::Scalar face_highlight_color, cv::Scalar body_highlight_color)
{
	/* Initialize the varaibles to be used in the for loop to improve efficiency */
	int bd_x; int bd_y;
	int bd_width; int bd_height;
	float average_face_depth;
	
	__focus_rects.clear();
	
	
	const int body_padding = 1;
	const int focus_padding = 2;
	for (int i=0; i < __faces.size(); i++)
	{
		cv::Rect face = __faces[i];
		
		/* Define center and elliptical dimensions of the face */
		cv::Point center( face.x + face.width * 0.5, face.y + face.height * 0.5);
		cv::ellipse(__rgb_frame, center, cv::Size(face.width*0.5, face.height*0.5), 0, 0, 360, face_highlight_color);
		
		/* From the face, derive the body dimensions and draw the body */
		int bd_x = face.x - (face.width * body_padding);
		int bd_y = face.y + face.height;
		int bd_width = constrainf(face.width + 2*(face.width * body_padding), 0, __rgb_frame.cols - 1);
		int bd_height =  __rgb_frame.rows - face.y - 1;
		cv::Rect body_bounding_rect(bd_x, bd_y, bd_width, bd_height);
		cv::rectangle(__rgb_frame, body_bounding_rect, body_highlight_color);
		
		/* Calculate the bounding box for the human */
		int focus_y = constrainf(bd_y - (face.height*2), 0, __rgb_frame.rows-1);
		int focus_height = __rgb_frame.rows - focus_y - 1;
		
		int focus_x = std::max(face.x - (face.width * focus_padding), 0);
		int focus_width = constrainf(face.width + 2*(face.width * focus_padding), 0, __rgb_frame.cols - focus_x - 1);
		
		cv::Rect focus_rect(focus_x, focus_y, focus_width, focus_height);
		
		if (focus_rect.x >= 0 && focus_rect.y >= 0 && focus_rect.width < __rgb_frame.cols && focus_rect.height < __rgb_frame.rows)
		{
			__focus_rects.push_back(focus_rect);
		}
		else
		{
			std::cout << "focus_rect_error: " << std::endl;
			std::cout << "focus_x: "          << focus_x          << std::endl;
			std::cout << "focus_y: "          << focus_y          << std::endl;
			std::cout << "focus_width: "      << focus_width      << std::endl;
			std::cout << "focus_height: "     << focus_height     << std::endl;
			std::cout << "face_x: "           << face.x           << std::endl;
			std::cout << "face_y: "           << face.y           << std::endl;
			std::cout << "face_width: "       << face.width       << std::endl;
			std::cout << "face_height: "      << face.height      << std::endl;
			std::cout << "__rgb_frame.cols: " << __rgb_frame.cols << std::endl;
			std::cout << "__rgb_frame.rows: " << __rgb_frame.rows << std::endl << std::endl;
		}
	}
}

void ImageCapture::draw_contours(cv::Mat &frame)
{
	for (int i=0; i<__contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		cv:;drawContours(frame, __contours, i, color, 2, 8, __hierarchy, 0, cv::Point());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processing_node");
	ImageCapture my_capture_device;
	
	if (!my_capture_device.start())
	{
		return -1;
	}
	

	//my_capture_device.start_window();
	
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		my_capture_device.run_once();
		//my_capture_device.update_window();
		ros::spinOnce(); // *Allows callbacks to be processed
		loop_rate.sleep();
	}
	
	my_capture_device.stop();
	return 0;
}
