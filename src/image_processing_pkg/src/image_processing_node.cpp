#include "image_processing_node.h"

int main(int argc, char **argv)
{
    // load required files
    face_cascade.load(face_cascade_path);

    // start ros
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    processed_image_publisher = nh_.advertise<sensor_msgs::Image>(render_image_topic, 1);

    // start the camera
    blink::Camera my_capture_device = blink::Camera(0);
    my_capture_device.start();

    ros::Rate loop_rate(fps);
    while (ros::ok())
    {
        cv::Mat frame = my_capture_device.get_next_frame();
        cv::Mat view_frame = render_preview(frame);
        publish_image(view_frame);

        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop the camera
    my_capture_device.stop();
    return 0;
}