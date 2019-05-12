#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class Gyrodometry{
	private:
		/*node hangle*/
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_bias;
		/*publish*/
		ros::Publisher pub_odom;
		tf::TransformBroadcaster tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry odom2d_now;
		nav_msgs::Odometry odom2d_last;
		nav_msgs::Odometry odom3d_now;
		nav_msgs::Odometry odom3d_last;
		/*objects*/
		sensor_msgs::Imu bias;
		sensor_msgs::Imu imu_last;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		/*flags*/
		bool first_callback_odom = true;
		bool first_callback_imu = true;
		bool inipose_is_available = false;
		bool bias_is_available = false;
	public:
		Gyrodometry();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		void Publication(void);
};

Gyrodometry::Gyrodometry()
{
	sub_inipose = nh.subscribe("/initial_orientation", 1, &Gyrodometry::CallbackInipose, this);
	sub_odom = nh.subscribe("/odom", 1, &Gyrodometry::CallbackOdom, this);
	sub_imu = nh.subscribe("/imu/data", 1, &Gyrodometry::CallbackIMU, this);
	sub_bias = nh.subscribe("/imu/bias", 1, &Gyrodometry::CallbackBias, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/gyrodometry", 1);
	InitializeOdom(odom3d_now);
	InitializeOdom(odom3d_last);
}

void Gyrodometry::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	odom.child_frame_id = "/gyrodometry";
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void Gyrodometry::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		odom3d_now.pose.pose.orientation = *msg;
		inipose_is_available = true;
	} 
}

void Gyrodometry::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;

	/*2Dto3D*/
	if(!first_callback_odom){
		tf::Quaternion q_pose2d_last;
		tf::Quaternion q_pose3d_last;
		quaternionMsgToTF(odom2d_last.pose.pose.orientation, q_pose2d_last);
		quaternionMsgToTF(odom3d_last.pose.pose.orientation, q_pose3d_last);

		tf::Quaternion q_global_move2d(
			odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
			odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
			odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z,
			0.0
		);
		tf::Quaternion q_local_move2d = q_pose2d_last.inverse()*q_global_move2d*q_pose2d_last;
		tf::Quaternion q_global_move3d = q_pose3d_last*q_local_move2d*q_pose3d_last.inverse();

		odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + q_global_move3d.x();
		odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + q_global_move3d.y();
		odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + q_global_move3d.z();
	}

	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;

	Publication();
}

void Gyrodometry::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*Get dt*/
	time_imu_now = ros::Time::now();
	double dt;
	try{
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_imu_last = time_imu_now;

	/*PoseEstimation*/
	if(first_callback_imu)	dt = 0.0;
	else if(bias_is_available){
		double delta_r = (msg->angular_velocity.x + imu_last.angular_velocity.x)*dt/2.0;
		double delta_p = (msg->angular_velocity.y + imu_last.angular_velocity.y)*dt/2.0;
		double delta_y = (msg->angular_velocity.z + imu_last.angular_velocity.z)*dt/2.0;
		delta_r -= bias.angular_velocity.x*dt;
		delta_p -= bias.angular_velocity.y*dt;
		delta_y -= bias.angular_velocity.z*dt;

		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		tf::Quaternion q_pose3d_now;
		quaternionMsgToTF(odom3d_now.pose.pose.orientation, q_pose3d_now);
		q_pose3d_now = q_pose3d_now*q_relative_rotation;
		q_pose3d_now.normalize();
		quaternionTFToMsg(q_pose3d_now, odom3d_now.pose.pose.orientation);
	}

	imu_last = *msg;
	first_callback_imu = false;
}

void Gyrodometry::CallbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!bias_is_available){
		bias = *msg;
		bias_is_available = true;
	}
}

void Gyrodometry::Publication(void)
{
	/*publish*/
	odom3d_now.header.stamp = odom2d_now.header.stamp;
	pub_odom.publish(odom3d_now);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = odom2d_now.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/gyrodometry";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyrodometry");

	Gyrodometry gyrodometry;

	ros::spin();
}
