#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class Gyrodometry{
	private:
		/*node hangle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_inipose;
		ros::Subscriber _sub_odom;
		ros::Subscriber _sub_imu;
		ros::Subscriber _sub_bias;
		/*publish*/
		ros::Publisher _pub_odom;
		tf::TransformBroadcaster _tf_broadcaster;
		/*odom*/
		nav_msgs::Odometry _odom2d_last;
		nav_msgs::Odometry _odom3d_now;
		nav_msgs::Odometry _odom3d_last;
		/*objects*/
		sensor_msgs::Imu _bias;
		sensor_msgs::Imu _imu_last;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		/*flags*/
		bool _got_first_odom = false;
		bool _got_first_imu = false;
		bool _got_inipose = false;
		bool _got_bias = false;
		/*parameters*/
		bool _wait_inipose;
		bool _linear_vel_is_available;
		std::string _frame_id;
		std::string _child_frame_id;
	public:
		Gyrodometry();
		void initializeOdom(nav_msgs::Odometry& odom);
		void callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void callbackBias(const sensor_msgs::ImuConstPtr& msg);
		void transformLinVel(nav_msgs::Odometry odom2d_now, double dt);
		void transformAngVel(sensor_msgs::Imu imu, double dt);
		void publication(ros::Time stamp);
};

Gyrodometry::Gyrodometry()
	:_nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("wait_inipose", _wait_inipose, true);
	std::cout << "_wait_inipose = " << (bool)_wait_inipose << std::endl;
	_nhPrivate.param("linear_vel_is_available", _linear_vel_is_available, false);
	std::cout << "_linear_vel_is_available = " << (bool)_linear_vel_is_available << std::endl;
	_nhPrivate.param("frame_id", _frame_id, std::string("/odom"));
	std::cout << "_frame_id = " << _frame_id << std::endl;
	_nhPrivate.param("child_frame_id", _child_frame_id, std::string("/gyrodometry"));
	std::cout << "_child_frame_id = " << _child_frame_id << std::endl;
	/*subscriber*/
	_sub_inipose = _nh.subscribe("/initial_orientation", 1, &Gyrodometry::callbackIniPose, this);
	_sub_odom = _nh.subscribe("/odom", 1, &Gyrodometry::callbackOdom, this);
	_sub_imu = _nh.subscribe("/imu/data", 1, &Gyrodometry::callbackIMU, this);
	_sub_bias = _nh.subscribe("/imu/_bias", 1, &Gyrodometry::callbackBias, this);
	/*publisher*/
	_pub_odom = _nh.advertise<nav_msgs::Odometry>("/gyrodometry", 1);
	/*initialize*/
	initializeOdom(_odom3d_now);
	initializeOdom(_odom3d_last);
}

void Gyrodometry::initializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = _frame_id;
	odom.child_frame_id = _child_frame_id;
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;

	if(!_wait_inipose)	_got_inipose = true;
}

void Gyrodometry::callbackIniPose(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
	if(!_got_inipose){
		_odom3d_now.pose.pose.orientation = msg->quaternion;
		_got_inipose = true;
	} 
}

void Gyrodometry::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/*skip first callback*/
	if(!_got_first_odom){
		_odom2d_last = *msg;
		_got_first_odom = true;
		return;
	}
	/*get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _odom2d_last.header.stamp).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	if(_got_inipose){
		/*transform*/
		transformLinVel(*msg, dt);
		/*publication*/
		publication(msg->header.stamp);
	}
	/*reset*/
	_odom2d_last = *msg;
	_odom3d_last = _odom3d_now;
}

void Gyrodometry::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*skip first callback*/
	if(!_got_first_imu){
		_imu_last = *msg;
		_got_first_imu = true;
		return;
	}
	/*Get dt*/
	double dt;
	try{
		dt = (msg->header.stamp - _imu_last.header.stamp).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
		return;
	}
	if(_got_inipose){
		/*transform*/
		transformAngVel(*msg, dt);
		/*publication*/
		publication(msg->header.stamp);
	}
	/*reset*/
	_imu_last = *msg;
}

void Gyrodometry::callbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!_got_bias){
		_bias = *msg;
		_got_bias = true;
	}
}

void Gyrodometry::transformLinVel(nav_msgs::Odometry odom2d_now, double dt)
{
	tf::Quaternion q_global_move3d;
	if(_linear_vel_is_available){
		tf::Quaternion q_ori3d_now;
		quaternionMsgToTF(_odom3d_now.pose.pose.orientation, q_ori3d_now);
		tf::Quaternion q_local_move(
			odom2d_now.twist.twist.linear.x*dt,
			0.0,
			0.0,
			0.0
		);
		q_global_move3d = q_ori3d_now*q_local_move*q_ori3d_now.inverse();
	}
	else{
		tf::Quaternion q_ori2d_last;
		tf::Quaternion q_ori3d_last;
		quaternionMsgToTF(_odom2d_last.pose.pose.orientation, q_ori2d_last);
		quaternionMsgToTF(_odom3d_last.pose.pose.orientation, q_ori3d_last);
		tf::Quaternion q_global_move2d(
			odom2d_now.pose.pose.position.x - _odom2d_last.pose.pose.position.x,
			odom2d_now.pose.pose.position.y - _odom2d_last.pose.pose.position.y,
			odom2d_now.pose.pose.position.z - _odom2d_last.pose.pose.position.z,
			0.0
		);
		tf::Quaternion q_local_move = q_ori2d_last.inverse()*q_global_move2d*q_ori2d_last;
		q_global_move3d = q_ori3d_last*q_local_move*q_ori3d_last.inverse();
	}
	_odom3d_now.pose.pose.position.x += q_global_move3d.x();
	_odom3d_now.pose.pose.position.y += q_global_move3d.y();
	_odom3d_now.pose.pose.position.z += q_global_move3d.z();
}

void Gyrodometry::transformAngVel(sensor_msgs::Imu imu, double dt)
{
	double dr = (imu.angular_velocity.x + _imu_last.angular_velocity.x)*dt/2.0;
	double dp = (imu.angular_velocity.y + _imu_last.angular_velocity.y)*dt/2.0;
	double dy = (imu.angular_velocity.z + _imu_last.angular_velocity.z)*dt/2.0;
	if(_got_bias){
		dr -= _bias.angular_velocity.x*dt;
		dp -= _bias.angular_velocity.y*dt;
		dy -= _bias.angular_velocity.z*dt;
	}
	tf::Quaternion q_rel_rot = tf::createQuaternionFromRPY(dr, dp, dy);
	tf::Quaternion q_ori3d_now;
	quaternionMsgToTF(_odom3d_now.pose.pose.orientation, q_ori3d_now);
	q_ori3d_now = q_ori3d_now*q_rel_rot;
	q_ori3d_now.normalize();
	quaternionTFToMsg(q_ori3d_now, _odom3d_now.pose.pose.orientation);
}

void Gyrodometry::publication(ros::Time stamp)
{
	/*publish*/
	_odom3d_now.header.stamp = stamp;
	_pub_odom.publish(_odom3d_now);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = stamp;
	transform.header.frame_id = _frame_id;
	transform.child_frame_id = _child_frame_id;
	transform.transform.translation.x = _odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = _odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = _odom3d_now.pose.pose.position.z;
	transform.transform.rotation = _odom3d_now.pose.pose.orientation;
	_tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyrodometry");

	Gyrodometry gyrodometry;

	ros::spin();
}
