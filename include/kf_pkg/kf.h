#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class kf
{
public:
	kf(ros::NodeHandle nh)
	{
		message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, "orb_slam2_mono/pose", 1);
		message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
		typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry> approximate_policy;
		message_filters::Synchronizer<approximate_policy> sync(approximate_policy(10),pose_sub, odom_sub);
		sync.registerCallback(boost::bind(&kf::PoseCallBack, this,_1, _2));


		is_initialized_ = false;
	}

	~kf()
	{

	}

	bool IsInitialized()
	{
		return is_initialized_;
	}
	
	// 初始化滤波器
	void Initialization(Eigen::VectorXd x_in)
	{
		x_ = x_in;
		is_initialized_ = true;
	}
	// 预测
	void Prediction()
	{
		x_ = F_ * x_;
		Eigen::MatrixXd Ft = F_.transpose();
		P_ = F_ * P_ * Ft + Q_;
	}
    // 更新
	void MeasureUpdate(const Eigen::VectorXd &z)
	{
		Eigen::VectorXd y = z - H_ * x_;
		Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
		Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
		x_ = x_ + K * y;
		Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
		P_ = (I - K * H_) * P_; 
	}
	// 设置预测函数
	void setF(Eigen::MatrixXd F_in)
	{
		F_ = F_in;
	}

	void setP(Eigen::MatrixXd P_in)
	{
		P_ = P_in;
	}

	void setQ(Eigen::MatrixXd Q_in)
	{
		Q_ = Q_in;
	}

	void setH(Eigen::MatrixXd H_in)
	{
		H_ = H_in;
	}

	void setR(Eigen::MatrixXd R_in)
	{
		R_ = R_in;
	}

	Eigen::VectorXd getx_()
	{
		Eigen::VectorXd x_out = x_;
		return x_out;
	}

	void PoseCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose, const nav_msgs::Odometry::ConstPtr& odom)
	{
		Eigen::VectorXd measurement;
		measurement(0) = pose->pose.pose.position.x;
		measurement(1) = pose->pose.pose.position.y;
		measurement(2) = pose->pose.pose.position.z;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose->pose.pose.orientation,quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		measurement(3) = roll;
		measurement(4) = pitch;
		measurement(5) = yaw;

		cur_time = (pose->header.stamp).toSec();

		if (!IsInitialized())
		{
			//  status (x,y,theta,v,w)
			Eigen::VectorXd status_;
			status_(0) = pose->pose.pose.position.x;
			status_(1) = pose->pose.pose.position.y;
			status_(2) = yaw;
			status_(3) = odom->twist.twist.linear.x;
			status_(4) = odom->twist.twist.angular.z;

			Initialization(status_);
			last_time = cur_time;
			Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(5,5);
			setQ(Q_in);

			Eigen::MatrixXd H_in(3,5);
			H_in << 1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1, 0, 0;
			setH(H_in);
		}

		double delta_t = cur_time - last_time;
		last_time = cur_time;

		// set F
		Eigen::MatrixXd F_in(5,5);
		F_in << 1, 0, 0, delta_t, 0,
				0, 1, 0, delta_t, 0,
				0, 0, 1, 0, delta_t,
				0, 0, 0, 1, 0,
				0, 0, 0, 0, 1;
		setF(F_in);

		Eigen::MatrixXd P_in(5,5);
		P_in << pose->pose.covariance[0], 0, 0, 0, 0,
				0, pose->pose.covariance[6], 0, 0, 0,
				0, 0, pose->pose.covariance[35], 0,0,
				0, 0, 0, 0, 0,
				0, 0, 0, 0, 0;
		setP(P_in);

		// prediction
		Prediction();
		// update
		MeasureUpdate(measurement);

		Eigen::VectorXd tmp_x = getx_();
		geometry_msgs::PoseWithCovarianceStamped filter_pose;
		filter_pose.header = pose->header;
		filter_pose.pose.pose.position.x = tmp_x[0];
		filter_pose.pose.pose.position.y = tmp_x[1];
		filter_pose.pose.pose.position.z = tmp_x[2];
		geometry_msgs::Quaternion tmp_q = tf::createQuaternionMsgFromRollPitchYaw(tmp_x[3], tmp_x[4], tmp_x[5]);
		filter_pose.pose.pose.orientation = tmp_q;

		pose_pub.publish(filter_pose);
	}

	void OdomCallBack(const nav_msgs::Odometry::ConstPtr &odom)
	{

	}

private:

	bool is_initialized_;

	Eigen::VectorXd x_;

	Eigen::MatrixXd F_;

	Eigen::MatrixXd P_;
	
	Eigen::MatrixXd Q_;
	
	Eigen::MatrixXd H_;

	Eigen::MatrixXd R_;

	double last_time, cur_time;

	ros::Publisher pose_pub;
};