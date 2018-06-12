#ifndef __IMAGE_PROCESSING_NODE_H
#define __IMAGE_PROCESSING_NODE_H

#include "image_processing_pkg/ProcessedImage.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "ros/ros.h"

#include "opencv2/opencv.hpp"

#include <algorithm>
#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <math.h>       /* sqrt */

#include "pi_color.h"
#include "pi_video.h"
#include "pi_objects.h"
#include "pi_svg_parser.h"
#include "pi_color_space.h"

cv::RNG rng(12345);

template <class T> const T& constrainf(const T& x, const T& min_x, const T& max_x)
{
	return std::min(max_x, std::max(min_x, x));
}

template <class T>
const T& maxf(const T& x, const T& y)
{
    return x > y ? x : y;
}

template <class T>
const T& minf(const T& x, const T& y)
{
    return x < y ? x : y;
}

const std::string home_path = "/home/odroid/artbot_ws/src/image_processing_pkg";
const std::string face_cascade_path = home_path + "/parameters/haarcascades/haarcascade_frontalface_default.xml";
const std::string default_bg_path = home_path + "/images/background.jpg";

const std::string node_name = "image_processing_node";
const std::string render_image_topic = "/processed_image";

const int fps = 15;
const int face_cascade_min_neighbour = 5;

const double erosion_size = 2;
const double dilation_size = 1;
const double face_cascade_scale_factor = 1.3;
const double filtered_points_min_threshold = 5;

const cv::Size blur_size(3,3);
// const cv::Size blur_size(5,5);

cv::CascadeClassifier face_cascade;
sensor_msgs::Image    processed_image;
std_msgs::String      paths_as_string_msg;
ros::Publisher        processed_image_publisher;
ros::Publisher        paths_as_string_publisher;

const std::string mdx_logo_path = home_path + "/svg/logo.svg";
blink::SvgParser svg_parser;

bool get_mdx_logo_as_paths(std::vector<blink::Path> &paths_out)
{
    std::vector<blink::Path> paths;

    svg_parser = blink::SvgParser();

    // load svg file
    svg_parser.load_file(mdx_logo_path);
    svg_parser.extract_paths();
    svg_parser.derieve_sheet_size(0, 0, 0, 0, 1);
    svg_parser.get_paths(paths);

    double scale = 0.4608;

    for (int i=0; i<paths.size(); i++)
    {
        paths.at(i).scale(scale, scale);
        paths.at(i).translate(552, 0);
    }

    paths_out.insert(paths_out.end(), paths.begin(), paths.end());
}

double measure(cv::Point p1, cv::Point p2)
{
    return std::sqrt( std::pow((p2.x - p1.x), 2) + std::pow((p2.y - p1.y), 2) );
}

int medianf(cv::Mat img)
{
    img = img.reshape(0, 1);
    std::vector<int> vec_format;
    img.copyTo(vec_format);

    std::nth_element(vec_format.begin(), vec_format.begin() + vec_format.size()/2, vec_format.end());
    return vec_format[vec_format.size() / 2];
}

void auto_canny(cv::Mat& src, cv::Mat& dst, double sigma)
{
    // compute the median of the single channel pixel intensities
    double v = medianf(src);

    int low_threshold = (int)  maxf<double>((double) 0  , (1.0 - sigma) * v);
    int high_threshold = (int) constrainf<double>(2.0 * low_threshold, 0, 255);

    cv::Canny(src, dst, low_threshold, high_threshold, 3);
}

void auto_canny_simple(cv::Mat& src, cv::Mat& dst, double sigma)
{
    // compute the median of the single channel pixel intensities
    double v = medianf(src);

    int low_threshold = (int)  maxf<double>((double) 0  , (1.0 - sigma) * v);
    int high_threshold = (int) minf<double>((double) 255, (1.0 + sigma) * v);

    cv::Canny(src, dst, low_threshold, high_threshold, 3);
}

void filter_points(std::vector<cv::Point > &point_sequence, std::vector<cv::Point > &new_point_sequence, double min_distance = 30)
{
    if (point_sequence.size() <= 0)
    {
        return;
    }

    cv::Point previous_point = point_sequence.at(0);
    new_point_sequence.push_back(previous_point);

    for (int i=1; i<point_sequence.size(); i++)
    {
        cv::Point current_point = point_sequence.at(i);

        double distance = std::abs(measure(previous_point, current_point));
        if (distance < min_distance)
        {
            continue;
        }

        new_point_sequence.push_back(current_point);
        previous_point = current_point;
    }
}

void transfer_frame_to_frame(cv::Mat &input, cv::Mat &output, cv::Rect from_region, cv::Rect to_region)
{
    cv::Mat extract = *(new cv::Mat(input, from_region));
    extract.copyTo(output(to_region));
}

void remove_background(cv::Mat &ref, cv::Mat &src, cv::Mat &dst, int focus_padding=1)
{
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(ref, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

    if (faces.size() <= 0)
    {
        dst = src.clone();
        return;
    }

    const int max_height = std::min(src.rows - 1, dst.rows - 1);
    const int max_width  = std::min(src.cols - 1, dst.rows - 1);

    for (int i=0; i<faces.size(); i++)
    {
        cv::Rect face = faces[i];

        /* Define center of the face */
        cv::Point center( face.x + face.width * 0.5, face.y + face.height * 0.5);
        
        /* Define body rect */
        int body_y = constrainf(face.y + face.height, 0, max_height);
        int body_height = max_height - body_y;

        int body_x = face.x - (face.width * focus_padding);
        int body_width = constrainf(face.width + 2*(face.width * focus_padding), 0, std::min(max_width - body_x, max_width));
        body_x = std::max(body_x, 0);

        cv::Rect body_rect(body_x, body_y, body_width, body_height);

        /* Add a little space around the face to include the head */
        double width_extra_percent = 0.3;
        double height_extra_percent = 0.12;
        int extra_face_width = face.width * width_extra_percent;
        int extra_face_height = face.height * height_extra_percent;

        face.width = constrainf(face.width + extra_face_width, 1, max_width - 1);
        face.height = constrainf(face.height + extra_face_height, 1, max_height - 1);
        face.x = constrainf(face.x - (extra_face_width / 2), 0, max_width - face.width);
        face.y = constrainf(face.y - extra_face_height, 0, max_height - face.height);

        transfer_frame_to_frame(src, dst, face, face);
        transfer_frame_to_frame(src, dst, body_rect, body_rect);
    }
}

void remove_background_body(cv::Mat &ref, cv::Mat &src, cv::Mat &dst, int focus_padding=1)
{
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(ref, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

    if (faces.size() <= 0)
    {
        dst = src.clone();
        return;
    }

    const int max_height = std::min(src.rows - 1, dst.rows - 1);
    const int max_width  = std::min(src.cols - 1, dst.rows - 1);

    for (int i=0; i<faces.size(); i++)
    {
        cv::Rect face = faces[i];

        /* Define body rect */
        int body_y = constrainf(face.y + face.height, 0, max_height);
        int body_height = max_height - body_y;

        int body_x = face.x - (face.width * focus_padding);
        int body_width = constrainf(face.width + 2*(face.width * focus_padding), 0, std::min(max_width - body_x, max_width));
        body_x = std::max(body_x, 0);

        cv::Rect body_rect(body_x, body_y, body_width, body_height);
        transfer_frame_to_frame(src, dst, body_rect, body_rect);
    }
}

void remove_background_faces(cv::Mat &ref, cv::Mat &src, cv::Mat &dst, int focus_padding=1)
{
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(ref, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

    if (faces.size() <= 0)
    {
        dst = src.clone();
        return;
    }

    const int max_height = std::min(src.rows - 1, dst.rows - 1);
    const int max_width  = std::min(src.cols - 1, dst.rows - 1);

    for (int i=0; i<faces.size(); i++)
    {
        cv::Rect face = faces[i];

        /* Add a little space around the face to include the head */
        double width_extra_percent = 0.3;
        double height_extra_percent = 0.12;
        int extra_face_width = face.width * width_extra_percent;
        int extra_face_height = face.height * height_extra_percent;

        face.width = constrainf(face.width + extra_face_width, 1, max_width - 1);
        face.height = constrainf(face.height + extra_face_height, 1, max_height - 1);
        face.x = constrainf(face.x - (extra_face_width / 2), 0, max_width - face.width);
        face.y = constrainf(face.y - extra_face_height, 0, max_height - face.height);

        transfer_frame_to_frame(src, dst, face, face);
    }
}

void reserve_logo_space(cv::Mat &src, cv::Mat &dst)
{
    // cv::Rect logo_space(0,380,100,100);
    
    cv::Rect part_a(0,0,540,100);
    cv::Rect part_b(0,100,640,380);

    transfer_frame_to_frame(src, dst, part_a, part_a);
    transfer_frame_to_frame(src, dst, part_b, part_b);
}

void publish_image(cv::Mat &frame)
{
    static int counter = 0;

    cv::Mat dst = cv::Mat(frame.rows, frame.cols, CV_8UC3);
    cv::flip(frame, dst, 1);

	try
	{
		cv_bridge::CvImage out_msg;
		out_msg.header.frame_id = "camera_frame";
		out_msg.header.seq = counter++;
		out_msg.header.stamp = ros::Time::now();
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = frame;

		sensor_msgs::ImagePtr im_msg = out_msg.toImageMsg();
		processed_image = *im_msg;
		processed_image_publisher.publish(processed_image);
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

void publish_paths_as_string(std::string paths_as_string)
{
    paths_as_string_msg.data = paths_as_string;
    paths_as_string_publisher.publish(paths_as_string_msg);
}

int render_preview(cv::Mat &frame, cv::Mat &view_frame, int focus_padding=1)
{
    std::vector<cv::Rect> faces;
    face_cascade.detectMultiScale(frame, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

    const int max_height = frame.rows - 1;
    const int max_width  = frame.cols - 1;

    view_frame = frame.clone();
    // view_frame.setTo(cv::Scalar(0, 0, 0));

    blink::Pathfinder pathfinder;
    for (int i=0; i<faces.size(); i++)
    {
        cv::Rect face = faces[i];

        int pad = 0.2 * face.height;
        face.y = constrainf(face.y - pad, 0, max_height);
        face.height = constrainf(face.height + pad, 0, max_height);

        /* Define center of the face */
        cv::Point center( face.x + face.width * 0.5, face.y + face.height * 0.5);
        double diameter = std::sqrt((face.width * face.width) + (face.height * face.height));

        /* Define body rect */
        int body_x = face.x - (face.width * focus_padding);
        int body_width = constrainf(face.width + 2*(face.width * focus_padding), 0, std::min(max_width - body_x, max_width));
        body_x = std::max(body_x, 0);

        int body_y = constrainf(face.y + face.height, 0, max_height);
        int body_height = max_height - body_y;

        cv::Rect body_rect(body_x, body_y, body_width, body_height);

        /* Add a little space around the face to include the head */
        double width_extra_percent = 0.3;
        double height_extra_percent = 0.12;
        int extra_face_width = face.width * width_extra_percent;
        int extra_face_height = face.height * height_extra_percent;

        face.width = constrainf(face.width + extra_face_width, 1, max_width - 1);
        face.height = constrainf(face.height + extra_face_height, 1, max_height - 1);
        face.x = constrainf(face.x - (extra_face_width / 2), 0, max_width - face.width);
        face.y = constrainf(face.y - extra_face_height, 0, max_height - face.height);
        
        blink::Path path_1 = blink::Path(face);
        blink::Path path_2 = blink::Path(body_rect);
        blink::Path new_path = pathfinder.unite(path_2, path_1);
        new_path.draw(view_frame, cv::Scalar(0, 255, 0));
    }

    return faces.size();
}

std::string render_final(cv::Mat &frame, cv::Mat &view_frame, cv::Size blur_kernel_size, int low_threshold=50, int high_threshold=150, int canny_kernel_size=3)
{
    cv::Mat gray_frame;
    cv::cvtColor( frame, gray_frame, CV_BGR2GRAY );

    cv::Mat blur_frame;
    cv::GaussianBlur(gray_frame, blur_frame, blur_kernel_size, 0, 0);

    cv::Mat edge_frame;
    auto_canny(gray_frame, edge_frame, 0.5);

    cv::Mat dilate_frame;
    cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size + 1), cv::Point( dilation_size, dilation_size ) );
    cv::dilate(edge_frame, dilate_frame, dilate_element);

    cv::Mat new_frame = cv::Mat::zeros(dilate_frame.size(), dilate_frame.type());
    remove_background_body(frame, dilate_frame, new_frame);

    cv::Mat new_frame_2 = cv::Mat::zeros(new_frame.size(), new_frame.type());
    reserve_logo_space(new_frame, new_frame_2);

    // cv::Mat erode_frame;
    // cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*erosion_size + 1, 2*erosion_size + 1), cv::Point( erosion_size, erosion_size ) );
    // cv::erode(new_frame_2, erode_frame, erode_element);

    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(new_frame_2, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    /* additional edges for the face */
    auto_canny_simple(gray_frame, edge_frame, 0.5);

    new_frame = cv::Mat::zeros(edge_frame.size(), edge_frame.type());
    remove_background_faces(frame, edge_frame, new_frame);

    std::vector<cv::Vec4i> additional_hierarchy;
    std::vector<std::vector<cv::Point> > additional_contours;
    cv::findContours(new_frame, additional_contours, additional_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    contours.insert(contours.end(), additional_contours.begin(), additional_contours.end() );

    view_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
    view_frame.setTo(cv::Scalar(255,255,255));

    int wasted_paths_counter = 0;

    std::vector<blink::Path> paths;
    std::stringstream ss; ss << "[" ;
    bool can_add_comma = false;

    get_mdx_logo_as_paths(paths);

    for (int i=0; i<paths.size(); i++)
    {
        blink::Path path = paths.at(i);

        if (can_add_comma)
        {
            ss << "," << path.get_as_string();
        }
        else
        {
           ss << path.get_as_string();
           can_add_comma = true;
        }

        cv::Scalar color = cv::Scalar(0, 0, 0);
        path.draw(view_frame, color);
    }

    int max_index = contours.size() - 1;
    for (int i=0; i<max_index; i++)
    {
        std::vector<cv::Point> contour = contours.at(i);
        blink::Path path = blink::Path(contour, true);
        if (path.rect_info.perimeter < 30)
        {
            wasted_paths_counter++;
            continue;
        }

        ss << "," << path.get_as_string();
        
        // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::Scalar color = cv::Scalar(0, 0, 0);
        path.draw(view_frame, color);
    }

    if (max_index >= 0)
    {
        std::vector<cv::Point> contour = contours.at(max_index);

        blink::Path path = blink::Path(contour, true);
        if (path.rect_info.perimeter >= 30)
        // if (path.rect_info.perimeter >= 1)
        {
            ss << ","  << path.get_as_string() << "]";
            // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            cv::Scalar color = cv::Scalar(0, 0, 0);
            path.draw(view_frame, color);
        }
        else
        {
            wasted_paths_counter++;
            ss << "]";
        }
    }
    else
    {
        ss << "]";
    }

    std::string paths_as_string = ss.str();

    std:cout << "wasted_paths_counter = " << wasted_paths_counter << " out of " << contours.size() << ", i.e " << (float) wasted_paths_counter/contours.size() * 100.0 << "%" << std::endl;
    // std::cout << "paths_as_string :" << paths_as_string << std::endl;

    // cv::cvtColor(edge_frame, view_frame, CV_RGB2GRAY);

    return paths_as_string;
}

// std::string render_final(cv::Mat &frame, cv::Mat &view_frame, cv::Size blur_kernel_size, int low_threshold=2, int high_threshold=75, int canny_kernel_size=3)
// {
//     // cv::Mat new_frame = cv::imread(default_bg_path, CV_LOAD_IMAGE_COLOR);
//     // if (new_frame.empty() || frame.rows != new_frame.rows || frame.cols != new_frame.cols)
//     // {
//     //     std::cout << "ERROR: incompactabile background" << std::endl;
//     //     std::cout << "rows: " << frame.rows << " cols: " << frame.cols << std::endl;
//     //     std::cout << "bg-rows: " << new_frame.rows << " bg-cols: " << new_frame.cols << std::endl;
//     //     new_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
//     // }

//     cv::Mat gray_frame;
//     cv::cvtColor( frame, gray_frame, CV_BGR2GRAY );

//     cv::Mat blur_frame;
//     cv::GaussianBlur(gray_frame, blur_frame, blur_kernel_size, 0, 0);

//     // equalize the constrast
//     // cv::equalizeHist( blur_frame, blur_frame );

//     cv::Mat edge_frame;
//     // cv::Canny(blur_frame, edge_frame, low_threshold, high_threshold, canny_kernel_size);
//     auto_canny(gray_frame, edge_frame, 0.4);

//     // cv::Mat dilate_frame;
//     // cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size + 1), cv::Point( dilation_size, dilation_size ) );
//     // cv::dilate(edge_frame, dilate_frame, dilate_element);

//     // cv::Mat erode_frame;
//     // cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*erosion_size + 1, 2*erosion_size + 1), cv::Point( erosion_size, erosion_size ) );
//     // cv::erode(dilate_frame, erode_frame, erode_element);

//     cv::Mat new_frame = cv::Mat::zeros(edge_frame.size(), edge_frame.type());
//     remove_background(frame, edge_frame, new_frame);

//     std::vector<cv::Vec4i> hierarchy;
//     std::vector<std::vector<cv::Point> > contours;
//     cv::findContours(new_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

//     view_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
//     view_frame.setTo(cv::Scalar(255,255,255));
//     // view_frame.setTo(cv::Scalar(255,255,255));
//     // view_frame = new_frame;

//     std::stringstream ss; ss << "[" ;

//     /*------------------------------------------------------------------------*/

//     // std::vector<cv::Rect> faces;
//     // blink::Pathfinder pathfinder;
//     // face_cascade.detectMultiScale(frame, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

//     // const int max_height = frame.rows - 1;
//     // const int max_width  = frame.cols - 1;
//     // std::vector<blink::Path> paths;
//     // const int shade_start_pad = 20;
//     // const int focus_padding = 1;
//     // int counter = 0;

//     // paths.reserve(faces.size());

//     // for (int i=0; i<faces.size(); i++)
//     // {
//     //     cv::Rect face = faces[i];

//     //     int pad = 0.2 * face.height;
//     //     face.y = constrainf(face.y - pad, 0, max_height);
//     //     face.height = constrainf(face.height + pad, 0, max_height);

//     //     /* Define center of the face */
//     //     cv::Point center( face.x + face.width * 0.5, face.y + face.height * 0.5);
//     //     double diameter = std::sqrt((face.width * face.width) + (face.height * face.height));

//     //     /* Define body rect */
//     //     int body_x = face.x - (face.width * focus_padding);
//     //     int body_width = constrainf(face.width + 2*(face.width * focus_padding), 0, std::min(max_width - body_x, max_width));
//     //     body_x = std::max(body_x, 0);

//     //     int body_y = constrainf(face.y + face.height, 0, max_height);
//     //     int body_height = max_height - body_y;

//     //     cv::Rect body_rect(body_x, body_y, body_width, body_height);
        
//     //     blink::Path path_1 = blink::Path(face);
//     //     blink::Path path_2 = blink::Path(body_rect);
//     //     blink::Path new_path = pathfinder.unite(path_2, path_1);

//     //     paths.push_back(new_path);
//     //     ss << new_path.get_as_string() << ",";
//     //     counter++;
//     //     new_path.draw(view_frame, cv::Scalar(0, 255, 0));
//     // }

//     // std::vector<blink::Line> shade_lines = pathfinder.shade_frame(
//     //     shade_start_pad,
//     //     shade_start_pad,
//     //     frame.cols - shade_start_pad,
//     //     frame.rows - shade_start_pad,
//     //     30,
//     //     0.,
//     //     paths
//     //     );

//     // for (int i=0; i<shade_lines.size(); i++)
//     // {
//     //     blink::Line line = shade_lines.at(i);
//     //     if (line.length < 10)
//     //     {
//     //         continue;
//     //     }

//     //     ss << line.get_as_string() << ",";
//     //     counter++;
//     //     line.draw(view_frame, cv::Scalar(0, 0, 255));
//     // }

//     /*------------------------------------------------------------------------*/
//     int max_index = contours.size() - 1;
//     for (int i=0; i<max_index; i++)
//     {
//         std::vector<cv::Point> contour = contours.at(i);
//         blink::Path path = blink::Path(contour, true);
//         // if (path.rect_info.perimeter < 15)
//         // {
//         //     continue;
//         // }

//         ss << path.get_as_string() << ",";
//         // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//         cv::Scalar color = cv::Scalar(0, 0, 0);
//         path.draw(view_frame, color);
//     }

//     if (max_index >= 0)
//     {
//         std::vector<cv::Point> contour = contours.at(max_index);

//         blink::Path path = blink::Path(contour, true);
//         // if (path.rect_info.perimeter >= 15)
//         if (path.rect_info.perimeter >= 1)
//         {
//             ss << path.get_as_string() << "]";
//             // cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//             cv::Scalar color = cv::Scalar(0, 0, 0);
//             path.draw(view_frame, color);
//         }
//         else
//         {
//             ss << "]";
//         }
//     }
//     else
//     {
//         ss << "]";
//     }

//     std::string paths_as_string = ss.str();

//     // std::cout << "paths_as_string :" << paths_as_string << std::endl;

//     // cv::cvtColor(edge_frame, view_frame, CV_RGB2GRAY);

//     return paths_as_string;
// }

#endif

// std::string render_final(cv::Mat &frame, cv::Mat &view_frame, cv::Size blur_kernel_size, int low_threshold=2, int high_threshold=75, int canny_kernel_size=3)
// {
//     // cv::Mat new_frame = cv::imread(default_bg_path, CV_LOAD_IMAGE_COLOR);
//     // if (new_frame.empty() || frame.rows != new_frame.rows || frame.cols != new_frame.cols)
//     // {
//     //     std::cout << "ERROR: incompactabile background" << std::endl;
//     //     std::cout << "rows: " << frame.rows << " cols: " << frame.cols << std::endl;
//     //     std::cout << "bg-rows: " << new_frame.rows << " bg-cols: " << new_frame.cols << std::endl;
//     //     new_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
//     // }
    
//     cv::Mat new_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
//     remove_background(frame, new_frame);

//     cv::Mat gray_frame;
//     cv::cvtColor( new_frame, gray_frame, CV_BGR2GRAY );

//     cv::Mat ref_gray_frame;
//     cv::cvtColor(frame, ref_gray_frame, CV_BGR2GRAY);

//     // equalize the constrast
//     // cv::equalizeHist( gray_frame, gray_frame );

//     cv::Mat blur_frame;
//     cv::GaussianBlur(gray_frame, blur_frame, blur_kernel_size, 0, 0);

//     // blink::simplify_image(blur_frame, blink::color_space_3d::color_array);

//     cv::Mat edge_frame;
//     // cv::Canny(blur_frame, edge_frame, low_threshold, high_threshold, canny_kernel_size);
//     auto_canny(ref_gray_frame, blur_frame, edge_frame, 0.33);

//     cv::Mat dilate_frame;
//     cv::Mat dilate_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*dilation_size + 1, 2*dilation_size + 1), cv::Point( dilation_size, dilation_size ) );
//     cv::dilate(edge_frame, dilate_frame, dilate_element);

//     cv::Mat erode_frame;
//     cv::Mat erode_element = getStructuringElement( cv::MORPH_RECT, cv::Size(2*erosion_size + 1, 2*erosion_size + 1), cv::Point( erosion_size, erosion_size ) );
//     cv::erode(dilate_frame, erode_frame, erode_element);

//     std::vector<cv::Vec4i> hierarchy;
//     std::vector<std::vector<cv::Point> > contours;
//     cv::findContours(erode_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

//     view_frame = cv::Mat::zeros(frame.size(), CV_8UC3);
//     view_frame.setTo(cv::Scalar(255,255,255));
//     // view_frame = new_frame;

//     std::stringstream ss; ss << "[" ;

//     /*------------------------------------------------------------------------*/

//     // std::vector<cv::Rect> faces;
//     // blink::Pathfinder pathfinder;
//     // face_cascade.detectMultiScale(frame, faces, face_cascade_scale_factor, face_cascade_min_neighbour);

//     // const int max_height = frame.rows - 1;
//     // const int max_width  = frame.cols - 1;
//     // std::vector<blink::Path> paths;
//     // const int shade_start_pad = 20;
//     // const int focus_padding = 1;
//     // int counter = 0;

//     // paths.reserve(faces.size());

//     // for (int i=0; i<faces.size(); i++)
//     // {
//     //     cv::Rect face = faces[i];

//     //     int pad = 0.2 * face.height;
//     //     face.y = constrainf(face.y - pad, 0, max_height);
//     //     face.height = constrainf(face.height + pad, 0, max_height);

//     //     /* Define center of the face */
//     //     cv::Point center( face.x + face.width * 0.5, face.y + face.height * 0.5);
//     //     double diameter = std::sqrt((face.width * face.width) + (face.height * face.height));

//     //     /* Define body rect */
//     //     int body_x = face.x - (face.width * focus_padding);
//     //     int body_width = constrainf(face.width + 2*(face.width * focus_padding), 0, std::min(max_width - body_x, max_width));
//     //     body_x = std::max(body_x, 0);

//     //     int body_y = constrainf(face.y + face.height, 0, max_height);
//     //     int body_height = max_height - body_y;

//     //     cv::Rect body_rect(body_x, body_y, body_width, body_height);
        
//     //     blink::Path path_1 = blink::Path(face);
//     //     blink::Path path_2 = blink::Path(body_rect);
//     //     blink::Path new_path = pathfinder.unite(path_2, path_1);

//     //     paths.push_back(new_path);
//     //     ss << new_path.get_as_string() << ",";
//     //     counter++;
//     //     new_path.draw(view_frame, cv::Scalar(0, 255, 0));
//     // }

//     // std::vector<blink::Line> shade_lines = pathfinder.shade_frame(
//     //     shade_start_pad,
//     //     shade_start_pad,
//     //     frame.cols - shade_start_pad,
//     //     frame.rows - shade_start_pad,
//     //     30,
//     //     0.,
//     //     paths
//     //     );

//     // for (int i=0; i<shade_lines.size(); i++)
//     // {
//     //     blink::Line line = shade_lines.at(i);
//     //     if (line.length < 10)
//     //     {
//     //         continue;
//     //     }

//     //     ss << line.get_as_string() << ",";
//     //     counter++;
//     //     line.draw(view_frame, cv::Scalar(0, 0, 255));
//     // }

//     /*------------------------------------------------------------------------*/
//     int max_index = contours.size() - 1;
//     for (int i=0; i<max_index; i++)
//     {
//         std::vector<cv::Point> contour = contours.at(i);
//         blink::Path path = blink::Path(contour, true);
//         // if (path.rect_info.perimeter < 15)
//         // {
//         //     continue;
//         // }

//         ss << path.get_as_string() << ",";
//         cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//         path.draw(view_frame, color);
//     }

//     if (max_index >= 0)
//     {
//         std::vector<cv::Point> contour = contours.at(max_index);

//         blink::Path path = blink::Path(contour, true);
//         // if (path.rect_info.perimeter >= 15)
//         if (path.rect_info.perimeter >= 1)
//         {
//             ss << path.get_as_string() << "]";
//             cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
//             path.draw(view_frame, color);
//         }
//         else
//         {
//             ss << "]";
//         }
//     }
//     else
//     {
//         ss << "]";
//     }

//     std::string paths_as_string = ss.str();

//     // std::cout << "paths_as_string :" << paths_as_string << std::endl;

//     // cv::cvtColor(edge_frame, view_frame, CV_RGB2GRAY);

//     return paths_as_string;
// }