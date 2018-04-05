#include "image_processing_node.h"
#include "state_change_action_server.h"


enum state
{
    state_normal,
    state_capture_image,
    state_render_preview
};
state execution_state = state_normal;


int main(int argc, char **argv)
{
    // load required files
    face_cascade.load(face_cascade_path);

    // initialise node
    ros::init(argc, argv, node_name);

    // initialise node handler
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    processed_image_publisher = nh_.advertise<sensor_msgs::Image>(render_image_topic, 1);
    paths_as_string_publisher = nh_.advertise<std_msgs::String>("path_as_string", 5);

    // start state_change_request_action server
    StateChangeRequestAction state_change_request(nh_, node_name);

    // start the camera
    blink::Camera my_capture_device = blink::Camera(0);
    my_capture_device.start();

    ros::Rate loop_rate(fps);
    while (ros::ok())
    {
        if (execution_state == state_normal)
        {
        }
        else if (execution_state == state_render_preview)
        {
            cv::Mat frame = my_capture_device.get_next_frame();
            cv::Mat view_frame;
            if (!frame.empty())
            {
                render_preview(frame, view_frame);
                publish_image(view_frame);
            }
        }
        else if (execution_state == state_capture_image)
        {
            ROS_INFO_STREAM("Starting image capture");

            double time_limit = 4.0;
            double start_time = ros::Time::now().toSec();

            bool render_image = false;

            while (ros::ok())
            {
                cv::Mat frame = my_capture_device.get_next_frame();
                cv::Mat view_frame;
                if ( !frame.empty() )
                {
                    int num_of_faces = render_preview(frame, view_frame);
                    publish_image(view_frame);

                    if (render_image || num_of_faces > 0)
                    {
                        ROS_INFO_STREAM("Found " << num_of_faces << " faces");
                        ROS_INFO_STREAM("Starting final render");

                        std::string paths_as_string = render_final(frame, view_frame, blur_size);
                        publish_image(view_frame);
                        publish_paths_as_string(paths_as_string);

                        break;
                    }
                }
                else if (render_image)
                {
                    ROS_INFO_STREAM("Fatal Error: requesting image capture when frame is empty");
                    break;
                }

                if ((double) ros::Time::now().toSec() - start_time >= time_limit)
                {
                    ROS_INFO_STREAM("Could not find a face, TIME UP");
                    render_image = true;
                }

                ros::spinOnce();
                loop_rate.sleep();
            }

            ROS_INFO_STREAM("Exiting image capture");

            // cv::Mat view_frame = render_final(frame, blur_size);
            execution_state = state_normal;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop the camera
    my_capture_device.stop();
    return 0;
}

void StateChangeRequestAction::executeCB(const image_processing_pkg::StateChangeRequestGoalConstPtr &goal)
{
    bool success = true;
    std::string state = goal->state;

    if (state == "state_normal")
    {
        ROS_INFO_STREAM(action_name_.c_str() << ": Entering State " << state.c_str());
        execution_state = state_normal;
    }
    else if (state == "state_capture_image")
    {
        ROS_INFO_STREAM(action_name_.c_str() << ": Entering State " << state.c_str());
        execution_state = state_capture_image;
    }
    else if (state == "state_render_preview")
    {
        ROS_INFO_STREAM(action_name_.c_str() << ": Entering State " << state.c_str());
        execution_state = state_render_preview;
    }
    else
    {
        success = false;
        ROS_INFO_STREAM(action_name_.c_str() << ": Could not identify State " << state.c_str());
        execution_state = state_normal;
    }

    feedback_.status = success;
    as_.publishFeedback(feedback_);

    if (success)
    {
        result_.status = feedback_.status;
        ROS_INFO_STREAM(action_name_.c_str() << ": Succeded");
        as_.setSucceeded(result_);
    }
}