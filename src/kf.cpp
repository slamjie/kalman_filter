#include "kf_pkg/kf.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf_node");

	ros::NodeHandle nh;
	kf m_kf;
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("orb_slam2_mono/pose", 1, &kf::PoseCallBack, &m_kf);

	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &kf::OdomCallBack, &kf);
	ros::spin();
	return 0;
}