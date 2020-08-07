#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <stdexcept>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdio>
#include <memory>
#include <string>
#include <array>

#include <powprof/async.h>

// globals for powprof
plnr::config*   _config;
plnr::sampler*	_sampler;
plnr::profiler*	_profiler;
plnr::pathn*	_model;
plnr::model_1layer* _model_1layer;
		
plnr::component _component("");

bool started = false;

void takeoffCallback(const std_msgs::Bool::ConstPtr& msg) {

	if (started) {
		ROS_INFO("powprofiler already running");
		return;
	}

	ROS_INFO("Stop called");

	// powprof initialization
	_config = new plnr::config();
	_sampler = new plnr::sampler_nano();
	
	// testing if the sampler works
	if (!_sampler->dryrun()) {
		ROS_ERROR("powprofiler does not work on this architecture");
	    return;
	}

	_profiler = new plnr::profiler(_config, _sampler);			
	size_t config_id = _config->add_configuration(_component, 0);
	ROS_INFO("config id => %zu", config_id);

	// starting powprof
	_model_1layer = new plnr::model_1layer(_config, _profiler, _component, config_id);
	_model_1layer->start();
	ROS_INFO("powprof started");

	started = true;
}

void landedCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (!started) {
		ROS_INFO("powprof not yet started");
		return;
	}

	ROS_INFO("Stop called");

	// stopping powprof
	_model_1layer->stop();
	ROS_INFO("powprof stoped");

	plnr::pathn* result = _model_1layer->get_model();
	ROS_INFO("detected avg power %f", result->avg().get(plnr::vectorn_flags::power));
	ROS_INFO("model saved in %s", result->save().c_str());
	started = false;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "powprof");
	ros::NodeHandle nh;

	ROS_INFO("powprof node");

	ros::Subscriber sub1 = nh.subscribe("/ros_ground_station/takeoff", 1000, takeoffCallback);
	ros::Subscriber sub2 = nh.subscribe("/ros_ground_station/land", 1000, landedCallback);

	while (ros::ok()) {
		ros::spinOnce();
	}

	return 0;
}
