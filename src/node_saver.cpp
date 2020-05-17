
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

using namespace message_filters;

std::vector<std::string> class_labels;
int count = 0;

void info_callback(const vision_msgs::VisionInfoConstPtr& inf) {
	
	// gettubng sine data from the topics in order to save the info about the detected classes (so the detection array returns just an id, if you need to find the name of the class you have to get the equivalent string strictly speaking)
	
	ROS_INFO("retriving class labels from parameters server");
	ROS_INFO("database location => %s", inf->database_location.c_str());

	ros::param::get(inf->database_location, class_labels);

	ROS_INFO("parameters saved");
}

void box_img_callback(const sensor_msgs::ImageConstPtr& msg, const vision_msgs::Detection2DArrayConstPtr& bbx) {

	// checking if vision info have been passed
	
	if (class_labels.size() < 1)
		return;

	ROS_INFO("detection callback");
	
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
  
	// drawing a bounding box per each detection, a rectangle above, and a text inside with the detected object class with confidence

	cv::RNG rng(12345);

	for (auto &det : bbx->detections) {

		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		ROS_INFO("detected a %s", class_labels.at(det.results[0].id).c_str());
		std::ostringstream title;
		title << class_labels.at(det.results[0].id) << " (" << std::fixed << std::setprecision(2) << det.results[0].score * 100 << "%)";

		cv::Mat overlay;
		cv_ptr->image.copyTo(overlay);
		cv::rectangle(overlay, 
			cv::Point(det.bbox.center.x - det.bbox.size_x / 2, det.bbox.center.y - det.bbox.size_y / 2),
			cv::Point(det.bbox.center.x + det.bbox.size_x / 2, det.bbox.center.y + det.bbox.size_y / 2),
			color,
			2
		);
		cv::rectangle(overlay,
                        cv::Point(det.bbox.center.x - det.bbox.size_x / 2, det.bbox.center.y - det.bbox.size_y / 2),
                        cv::Point(det.bbox.center.x + det.bbox.size_x / 2, (det.bbox.center.y - det.bbox.size_y / 2) + 32),
                        color,
                        -1
                );
		cv::putText(overlay, title.str(), cv::Point((det.bbox.center.x - det.bbox.size_x / 2) + 8, (det.bbox.center.y - det.bbox.size_y / 2) + 22), cv::FONT_HERSHEY_SIMPLEX, 0.56, cv::Scalar(255, 255, 255), 1);
		cv::addWeighted(overlay, 0.68, cv_ptr->image, 0.32, 0, cv_ptr->image);
	}

	// storing images locally
	
	std::string stored_path;
	struct timeval tv;
	imwrite(stored_path = "image_" + std::to_string(count++) + "-" + std::to_string(tv.tv_sec * 1000 + tv.tv_usec / 1000) + ".jpg", cv_ptr->image);

	// all stored :)
	
	ROS_INFO("image stored as %s", stored_path.c_str());
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "saver");

	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	// used to get the detections classes
	
	ros::Subscriber info_sub = nh.subscribe("/detectnet/vision_info", 1, info_callback);

	// syncronizing the two topics; an easy task as I modified the detections and image_raw topics in such a way that they have the same timestamp
	
	message_filters::Subscriber<vision_msgs::Detection2DArray> bounding_boxes_sub(nh, "/detectnet/detections", 1);
	message_filters::Subscriber<sensor_msgs::Image> image_raw_sub(nh, "/cv_camera/image_raw", 1);
	TimeSynchronizer<sensor_msgs::Image, vision_msgs::Detection2DArray> sync(image_raw_sub, bounding_boxes_sub, 1000);

	sync.registerCallback(boost::bind(&box_img_callback, _1, _2));

	ROS_INFO("node saver initilized and ready for processing detections");

	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

