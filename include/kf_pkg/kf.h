#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStampedWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>


class kf
{
public:
	kf()
	{
		is_initialized_ = false;
	}

	~kf()
	{

	}

	Eigen::VectorXd GetX()
	{
		return x_;
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

	void PoseCallBack(const geometry_msgs::PoseStampedWithCovariance::ConstPtr& pose)
	{
		Eigen::VectorXd measurement;
		measurement(0) = pose->position.x;
		measurement(1) = pose->position.y;
		measurement(2) = pose->position.z;
		tf::Quaternion quat;
		tf::quaternionMsgToTF(pose->orientation,quat);
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		measurement(3) = roll;
		measurement(4) = pitch;
		measurement(5) = yaw;

		cur_time = (pose->header.stamp).toSec();

		if (!IsInitialized())
		{
			Initialization(measurement);
			last_time = cur_time;

			Eigen::Matrix5d Q_in = Eigen::Matrix5d::Identity();
			kf.setQ(Q_in);

			Eigen::MatrixXd H_in(3,5);
			H_in << 1, 0, 0, 0, 0,
					0, 1, 0, 0, 0,
					0, 0, 1, 0, 0;
			kf.setH(H_in);
		}

		double delta_t = cur_time - last_time;
		last_time = cur_time;

		// set F
		Eigen::MatrixXd F_in(5,5);
		F_in << 1, 0, 0, delta_t, 0,
				0, 1, 0, delta_t, 0,
				0, 0, 1, 0, delta_t,
				0, 0, 0, 1, 0.,
				0, 0, 0, 0, 1;
		kf.setF(F_in);
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

};