#include "image_processing_node.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_processing_node");
	ImageCapture my_capture_device;

	if (!my_capture_device.start())
	{
		return -1;
	}


	//my_capture_device.start_window();

	ros::Rate loop_rate(15);
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