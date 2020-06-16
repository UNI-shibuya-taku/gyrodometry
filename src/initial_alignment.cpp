#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class InitialAlignment{
	private:
		/*node hangle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_imu;
		/*publish*/
		ros::Publisher _pub_quat;
		ros::Publisher _pub_bias;
		tf::TransformBroadcaster tf_broadcaster;
		/*const*/
		const int _data_size = 6;	//ax, ay, az, wx, wy, wz
		/*msg*/
		geometry_msgs::QuaternionStamped _ini_ori;
		sensor_msgs::Imu _bias;
		/*time*/
		ros::Time _start_stamp;
		/*vector*/
		Eigen::VectorXd _sum;
		Eigen::VectorXd _ave;
		/*object*/
		int _record_size = 0;
		/*flag*/
		bool _is_done = false;
		bool _is_fail = false;
		/*parameter*/
		bool _ini_ori_is_0001;
		std::string _frame_id;
		std::string _child_frame_id;
		int _min_record_size;
		double _max_duration;
		double _th_linear_deflection;
		double _th_angle_deflection;
	public:
		InitialAlignment();
		void callbackIMU(const sensor_msgs::ImuConstPtr& msg);
		bool isMoving(const Eigen::VectorXd& data);
		void setMsg(std_msgs::Header imu_header);
		void publication(void);
};

InitialAlignment::InitialAlignment()
	:_nhPrivate("~")
{
	std::cout << "IMU Initial Alignment: START" << std::endl;
	/*parameter*/
	_nhPrivate.param("_ini_ori_is_0001", _ini_ori_is_0001, false);
	std::cout << "_ini_ori_is_0001" << (bool)_ini_ori_is_0001 << std::endl;
	_nhPrivate.param("_frame_id", _frame_id, std::string("/odom"));
	std::cout << "_frame_id" << _frame_id << std::endl;
	_nhPrivate.param("_child_frame_id", _child_frame_id, std::string("/initial_orientation"));
	std::cout << "_child_frame_id" << _child_frame_id << std::endl;
	_nhPrivate.param("_min_record_size", _min_record_size, 100);
	std::cout << "_min_record_size" << _min_record_size << std::endl;
	_nhPrivate.param("_max_duration", _max_duration, 60.0);
	std::cout << "_max_duration" << _max_duration << std::endl;
	_nhPrivate.param("_th_linear_deflection", _th_linear_deflection, 0.03);
	std::cout << "_th_linear_deflection" << _th_linear_deflection << std::endl;
	_nhPrivate.param("_th_angle_deflection", _th_angle_deflection, 0.2);
	std::cout << "_th_angle_deflection" << _th_angle_deflection << std::endl;
	/*subscriber*/
	_sub_imu = _nh.subscribe("/imu/data", 1, &InitialAlignment::callbackIMU, this);
	/*publisher*/
	_pub_quat = _nh.advertise<geometry_msgs::Quaternion>("/initial_orientation", 1);
	_pub_bias = _nh.advertise<sensor_msgs::Imu>("/imu/bias", 1);
	/*initialize*/
	_sum = Eigen::VectorXd::Zero(_data_size);
}

void InitialAlignment::callbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	if(_is_done)	publication();
	if(_is_fail)	return;

	/*record*/
	Eigen::VectorXd data(_data_size);
	data <<
		msg->linear_acceleration.x,
		msg->linear_acceleration.y,
		msg->linear_acceleration.z,
		msg->angular_velocity.x,
		msg->angular_velocity.y,
		msg->angular_velocity.z;

	if(_record_size == 0){
		_start_stamp = msg->header.stamp;
	}
	else{
		if(isMoving(data)){
			if(_record_size > _min_record_size){
				_is_done = true;
				setMsg(msg->header);
				publication();
			}
			else{
				_is_fail = true;
				std::cout << "IMU Initial Alignment: FAIL" << std::endl;
			}
			return;
		}
	}

	/*update*/
	_sum += data;
	++_record_size;
	_ave = _sum/(double)_record_size;
	/*dropout*/
	double duration;
	try{
		duration = (msg->header.stamp - _start_stamp).toSec();
		if(duration > _max_duration){
			_is_done = true;
			setMsg(msg->header);
			publication();
		}
	}
	catch(std::runtime_error& ex){
		ROS_ERROR("Exception: [%s]", ex.what());
	}
}

bool InitialAlignment::isMoving(const Eigen::VectorXd& data)
{
	Eigen::VectorXd deflection = _ave - data;
	for(size_t i=0;i<deflection.size();++i){
		if(i < 3){
			if(abs(deflection(i)) > _th_linear_deflection){
				std::cout << "Moving: " << i << std::endl;
				return true;
			}
		}
		else{
			if(abs(deflection(i)) > _th_angle_deflection){
				std::cout << "Moving: " << i << std::endl;
				return true;
			}
		}
	}
	return false;
}

void InitialAlignment::setMsg(std_msgs::Header imu_header)
{
	/*bias*/
	_bias.header = imu_header;
	_bias.linear_acceleration.x = _ave(0);
	_bias.linear_acceleration.y = _ave(1);
	_bias.linear_acceleration.z = _ave(2);
	_bias.angular_velocity.x = _ave(3);
	_bias.angular_velocity.y = _ave(4);
	_bias.angular_velocity.z = _ave(5);
	/*initial orientation*/
	_ini_ori.header.frame_id = _frame_id;
	_ini_ori.header.stamp = imu_header.stamp;
	if(_ini_ori_is_0001){
		_ini_ori.quaternion.x = 0.0;
		_ini_ori.quaternion.y = 0.0;
		_ini_ori.quaternion.z = 0.0;
		_ini_ori.quaternion.w = 1.0;
	}
	else{
		double r = atan2(_ave(1), _ave(2));
		double p = atan2(_ave(0), sqrt(_ave(1)*_ave(1) + _ave(2)*_ave(2)));
		tf::Quaternion q = tf::createQuaternionFromRPY(r, p, 0.0);
		quaternionTFToMsg(q, _ini_ori.quaternion);
	}
	std::cout << "IMU Initial Alignment: DONE" << std::endl;
}

void InitialAlignment::publication(void)
{
	/*publish*/
	_pub_quat.publish(_ini_ori);
	_pub_bias.publish(_bias);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = _frame_id;
	transform.child_frame_id = _child_frame_id;
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;
	transform.transform.rotation = _ini_ori.quaternion;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "initial_alignment");

	InitialAlignment initial_alignment;

	ros::spin();
}
