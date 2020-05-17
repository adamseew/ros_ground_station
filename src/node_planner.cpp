
#include <mavros_msgs/WaypointReached.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <vector>

ros::WallTime start_;

std::string camera_rate = "/cv_camera/rate";
std::string ground_protocol = "/ros_ground_station/protocol";
std::string ground_rate = "/ros_ground_station/rate";
std::string http = "http";
std::string https = "https";

int mission_id;

void wp_callback(const mavros_msgs::WaypointReachedConstPtr& wp) {
	ros::NodeHandle nh;

	if (mission_id == 1) {
		if (wp->wp_seq <= 0) {
			nh.setParam(camera_rate, 0.5);
                	nh.setParam(ground_protocol, http);
                	nh.setParam(ground_rate, 0.5);
		} else if (wp->wp_seq >= 1 && wp->wp_seq <= 12) {
			nh.setParam(camera_rate, 7);
                        nh.setParam(ground_protocol, https);
                        nh.setParam(ground_rate, 0.1);
		} else if (wp->wp_seq >= 13 && wp->wp_seq <= 22) {
			nh.setParam(camera_rate, 2);
                        nh.setParam(ground_protocol, https);
                        nh.setParam(ground_rate, 0.1);
		} else if (wp->wp_seq >= 23) {
			nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 5);
		}


	} else if (mission_id == 2) {
		if (wp->wp_seq <= 0) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 0.5);
                } else if (wp->wp_seq >= 1 && wp->wp_seq <= 12) {
			if (wp->wp_seq >= 2 && wp->wp_seq <= 5 || wp->wp_seq >= 8 && wp->wp_seq <= 11) {
				nh.setParam(camera_rate, 7);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
			} else {
                        	nh.setParam(camera_rate, 3);
                        	nh.setParam(ground_protocol, https);
                        	nh.setParam(ground_rate, 1);
			}
                } else if (wp->wp_seq >= 13 && wp->wp_seq <= 22) {
                        if (wp->wp_seq >= 13 && wp->wp_seq <= 15 || wp->wp_seq >= 18 && wp->wp_seq <= 19 || wp->wp_seq >= 20 && wp->wp_seq <= 21) {
                                nh.setParam(camera_rate, 3);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
                        } else {
                                nh.setParam(camera_rate, 1);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 1);
                        }

		} else if (wp->wp_seq >= 23) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 5);
                }


	} else if (mission_id == 3 && mission_id == 6) {
		nh.setParam(camera_rate, 7);
                nh.setParam(ground_protocol, http);
                nh.setParam(ground_rate, 1);


	} else if (mission_id == 4) {
                if (wp->wp_seq <= 0) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 0.5);
                } else if (wp->wp_seq >= 1 && wp->wp_seq <= 6) {
                        nh.setParam(camera_rate, 7);
                        nh.setParam(ground_protocol, https);
                        nh.setParam(ground_rate, 0.1);
                } else if (wp->wp_seq >= 7 && wp->wp_seq <= 23) {
                        nh.setParam(camera_rate, 2);
                        nh.setParam(ground_protocol, https);
                        nh.setParam(ground_rate, 0.1);
                } else if (wp->wp_seq >= 24) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 5);
                }


        } else if (mission_id == 5) {
                if (wp->wp_seq <= 0) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 0.5);
                } else if (wp->wp_seq >= 1 && wp->wp_seq <= 6) {
                        if (wp->wp_seq >= 2 && wp->wp_seq <= 4) {
                                nh.setParam(camera_rate, 7);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
                        } else {
                                nh.setParam(camera_rate, 3);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 1);
                        }
                } else if (wp->wp_seq >= 7 && wp->wp_seq <= 23) {
                        if (wp->wp_seq >= 7 && wp->wp_seq <= 9 || wp->wp_seq >= 12 && wp->wp_seq <= 14 || wp->wp_seq >= 17 && wp->wp_seq <= 19 || wp->wp_seq >= 21 && wp->wp_seq <= 23) {
                                nh.setParam(camera_rate, 3);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
                        } else {
                                nh.setParam(camera_rate, 1);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 1);
                        }

                } else if (wp->wp_seq >= 24) {
                        nh.setParam(camera_rate, 0.5);
                        nh.setParam(ground_protocol, http);
                        nh.setParam(ground_rate, 5);
                }


        }

	ROS_INFO("config changed, wp_seq => %d", wp->wp_seq);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "planner");

	ros::NodeHandle nh;

	ros::Rate loop_rate(10);

	ros::param::get("/ros_ground_station/mission", mission_id);

	ros::Subscriber wp_sub = nh.subscribe("/mavros/mission/reached", 1, wp_callback);

	ROS_INFO("node planner initialized and ready to plan mission");

	ROS_INFO("mission id => %d", mission_id);

	start_ = ros::WallTime::now();

	while (ros::ok()) {

		ros::spinOnce();
		loop_rate.sleep();

		if (mission_id == 7) {
			ros::WallTime end_ = ros::WallTime::now();

			double execution_time = (end_ - start_).toNSec() * 1e-6;
			execution_time /= 1000;
			if (execution_time < 180) {
				nh.setParam(camera_rate, 0.5);
	                        nh.setParam(ground_protocol, http);
        	                nh.setParam(ground_rate, 0.5);
			} else if (execution_time >= 180 && execution_time <= 360) {
				nh.setParam(camera_rate, 7);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
			} else {
				nh.setParam(camera_rate, 1);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 1);
			}
		} else if (mission_id == 8) {
                        ros::WallTime end_ = ros::WallTime::now();

                        double execution_time = (end_ - start_).toNSec() * 1e-6;
                        execution_time /= 1000;
                        if (execution_time < 180) {
                                nh.setParam(camera_rate, 0.5);
                                nh.setParam(ground_protocol, http);
                                nh.setParam(ground_rate, 0.5);
                        } else if (execution_time >= 180 && execution_time <= 240) {
                                nh.setParam(camera_rate, 7);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
                        } else if (execution_time >= 240 && execution_time <= 360) {
                                nh.setParam(camera_rate, 3);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 0.1);
                        } else {
                                nh.setParam(camera_rate, 1);
                                nh.setParam(ground_protocol, https);
                                nh.setParam(ground_rate, 1);
                        }

		}
	}

	return 0;
}


