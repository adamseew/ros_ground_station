
#include <signal.h>
#include <iostream>
#include <vector>
#include <sstream>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/VisionInfo.h>

#include <powprof/async.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <vision_msgs/VisionInfo.h>

#include <sys/time.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#define BOOST_RANGE_ENABLE_CONCEPT_ASSERT 0
#include <boost/filesystem.hpp>
#include <boost/range/adaptors.hpp>

namespace bfs = boost::filesystem;
namespace ba = boost::adaptors;

std::string exec(const char* cmd) {
	
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

	if (!pipe)
		throw std::runtime_error("popen failed");
	
	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
		result += buffer.data();
	}

	return result;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "sender");
	ros::NodeHandle nh;

	std::string protocol = "http";
	std::string old_protocol = "";
	bool is_https = true;
	double rate = 1;
	double old_rate = -1;

	// so each second we will try to send all the image_[something].jpg to the ground station :)
	
	ROS_INFO("node sender initialized and ready to send images to the ground station");

	int count = 0;
	while (ros::ok()) {

		ros::param::get("/ros_ground_station/protocol", protocol);
        	ros::param::get("/ros_ground_station/rate", rate);

        	// so each second we will try to send all the image_[something].jpg to the ground station :)


		if (protocol != old_protocol) {
			old_protocol = protocol;
        		if (protocol != "https") {
                		is_https = false;
                		ROS_INFO("protocol set as http");
        		} else { 
				is_https = true;
                		ROS_INFO("protocol set as https");
			}
		}
		if (rate != old_rate) {
			old_rate = rate;
			ROS_INFO("rate set to %f Hz", rate);
		}

        	ros::Rate loop_rate(rate);
		ros::spinOnce();
		loop_rate.sleep();

		// getting all the images starting with image in the catkin workspace

		const std::string target_path("/home/user/catkin_ws/");
		const boost::regex my_filter("image.*\.jpg");
		boost::smatch what;

		std::vector<std::string> images;

		for 	(auto &entry: boost::make_iterator_range(bfs::directory_iterator(target_path), {})
			| ba::filtered(static_cast<bool (*)(const bfs::path &)>(&bfs::is_regular_file))
			| ba::filtered([&](const bfs::path &path){ return boost::regex_match(path.filename().string(), what, my_filter); })
			) {

			
			// images matching the pattern "image*.jpg".
  
			images.push_back(entry.path().filename().string());
		}

		if (images.size() > 0) {
			
			ROS_INFO("ready to send curl commands");
			ROS_INFO("images count => %d", images.size());
			
			for (auto image : images) {

				std::string curl_command = "curl ";
				curl_command += "-F \"file=@" + target_path + image + "\" ";
				if (is_https)
					curl_command += "-k https://192.168.43.3/fromagent.php";
				else
					curl_command += "http://192.168.43.3/fromagent.php";

				if (exec(curl_command.c_str()).find("ack") != std::string::npos) {
			
					ROS_INFO("transfer ack");
				
					// deleting image that has been sent correctly
				
					boost::filesystem::remove(image);					
					ROS_INFO("%s successfully sent, db purged", image.c_str());

				} else {
					ROS_INFO("something went wrong with the conncection");
				}
			}
		} else  
			ROS_INFO("nothing to send this time");

		++count;
	}
	return 0;
}

