#include "kf_pkg/kf.h"

void filter(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose, const nav_msgs::OdometryConstPtr& odom)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kf_node");

	ros::NodeHandle nh;
	kf m_kf(nh);

	ros::spin();
	return 0;
}