#ifndef BLINK_PI_VIDEO_H
#define BLINK_PI_VIDEO_H

#include <vector>
#include <string>
#include "pi_color.h"
#include "opencv2/opencv.hpp"

using namespace std;

namespace blink
{

class Camera
{
	bool              _started;
	cv::VideoCapture  _cap;
	int               _device_id;
	cv::Mat           _frame;

public:
	Camera();
	Camera(int device_id);

	bool start();
	bool stop();
	cv::Mat get_next_frame();
	cv::Mat get_prev_frame();
};

class Viewer
{
	bool    _started;
	string  _title;
	int     _fms;

public:
	Viewer();
	Viewer(int fps, string title);

	bool start();
	bool stop();
	int show_frame(cv::Mat frame);
};

class StaticImage
{
	bool    _started;
	cv::Mat     _frame;
	string  _image_name;

public:
	StaticImage();
	StaticImage(string image_name);

	bool start();
	bool stop();
	cv::Mat get_next_frame();
	cv::Mat get_prev_frame();
};

void simplify_image(cv::Mat &frame, std::vector<RGBColor> color_array)
{
	double r; double b; double g;
	for (int row=0; row<frame.rows; row++)
	{
		for (int col=0; col<frame.cols; col++)
		{
			cv::Vec3b pixel = frame.at<cv::Vec3b>(row, col);
			b = (double) (pixel[0] / 255.0);
			g = (double) (pixel[1] / 255.0);
			r = (double) (pixel[2] / 255.0);
			RGBColor color = RGBColor(r, g, b);

			color = simplify(color, color_array);
			pixel[0] = (unsigned char)(color.rgb_color.b * 255);
			pixel[1] = (unsigned char)(color.rgb_color.g * 255);
			pixel[2] = (unsigned char)(color.rgb_color.r * 255);
			frame.at<cv::Vec3b>(row, col) = pixel;
		}
	}
}

Camera::Camera()
{
	_started = false;
	_device_id = 1;
}

Camera::Camera(int device_id)
{
	_started = false;
	_device_id = device_id;
}

bool Camera::start()
{
	if (!_started)
	{
		_started = true;
		_cap = *(new cv::VideoCapture(_device_id));
	}

	return _started;
}

bool Camera::stop()
{
	if (_started)
	{
		_started = false;
		_cap.release();
	}

	return !_started;
}

cv::Mat Camera::get_next_frame()
{
	if (_started)
	{
		_cap >> _frame;
	}

	return _frame;
}

cv::Mat Camera::get_prev_frame()
{
	return _frame;
}

Viewer::Viewer()
{
	_started = false;
	_title = "frame";

	double fps = 30;
	_fms = 1000.0/fps;
}

Viewer::Viewer(int fps, string title)
{
	_started = false;
	_title = title;
	_fms = 1000.0/fps;
}

bool Viewer::start()
{
	if (!_started)
	{
		_started = true;
		cv::namedWindow(_title, CV_WINDOW_AUTOSIZE);
	}

	return _started;
}

bool Viewer::stop()
{
	if (_started)
	{
		_started = false;
		cv::destroyAllWindows();
	}

	return !_started;
}

int Viewer::show_frame(cv::Mat frame)
{
	if (_started)
	{
		cv::imshow(_title, frame);
		return cv::waitKey(_fms);
	}

	return -1;
}

StaticImage::StaticImage()
{
	_started = false;
	_image_name = "";
}

StaticImage::StaticImage(string image_name)
{
	_started = false;
	_image_name = image_name;
}

bool StaticImage::start()
{
	if (_image_name.length() > 0)
	{
		_frame = cv::imread(_image_name);
	}
	return _started;
}

bool StaticImage::stop()
{
	_started = false;
	return !_started;
}

cv::Mat StaticImage::get_next_frame()
{
	return _frame;
}

cv::Mat StaticImage::get_prev_frame()
{
	return _frame;
}

}






#endif /*BLINK_PI_VIDEO_H*/