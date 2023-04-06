
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <vision_msgs/Detection2DArray.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vision_msgs/VisionInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <powprof/async.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>

using namespace message_filters;

class Saver {

public:
    Saver() {

        // used to get the detections classes

        info_sub = node.subscribe<vision_msgs::VisionInfo>("/detectnet/vision_info", 1, &Saver::info_callback, this);

        // detection

        bounding_boxes_sub.subscribe(node, "/detectnet/detections", 1);
        image_raw_sub.subscribe(node, "/cv_camera/image_raw", 1);
	sync_.reset(new Sync(SyncPolicy(10), image_raw_sub, bounding_boxes_sub));
	sync_->registerCallback(boost::bind(&Saver::box_img_callback, this, _1, _2));
    }
	
    void info_callback(const vision_msgs::VisionInfoConstPtr& inf) {
	
        // saving info about the detected classes 
        // the detection array returns just an id, if ou need to find the 
        // name of the class you have to get the equivalent string 

        ROS_INFO("retriving class labels from parameters server");
        ROS_INFO("database location => %s", inf->database_location.c_str());    	ros::param::get(inf->database_location, class_labels);
        
        ROS_INFO("parameters saved");
    }

    void box_img_callback(const sensor_msgs::ImageConstPtr& msg, const vision_msgs::Detection2DArrayConstPtr& bbx) {

        // checking if vision info has been passed

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

private:
    ros::NodeHandle node;
    std::vector<std::string> class_labels;
    int count = 0;

    message_filters::Subscriber<vision_msgs::Detection2DArray> bounding_boxes_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_raw_sub;
    ros::Subscriber info_sub;
    typedef sync_policies::ApproximateTime<sensor_msgs::Image,vision_msgs::Detection2DArray> SyncPolicy;
    typedef Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv) {

	ros::init(argc, argv, "saver");

	Saver saver_;
	ROS_INFO("node saver initilized and ready for processing detections");

	ros::spin();

	return 0;
}

