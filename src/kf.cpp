#include "kf_pkg/kf.h"

void filter(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf_node");

	ros::NodeHandle nh;
	kf m_kf(nh);
	
	// ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("orb_slam2_mono/pose", 1, &kf::PoseCallBack, &m_kf);

	// Eigen::VectorXd tmp_x = m_kf.getx_();

	// ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &kf::OdomCallBack, &m_kf);
	ros::spin();
	return 0;
}