
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <powprof/async.h>
#include <ros/ros.h>
#include <signal.h>
#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "receiver");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {

		// TODO channel in ground-station -> drone direction

		ros::spinOnce();
	}

	return 0;
}

