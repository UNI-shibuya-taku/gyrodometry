#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ImuInitialAlignment{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_imu;
		/*publish*/
		ros::Publisher pub_inipose;
		ros::Publisher pub_bias;
		tf::TransformBroadcaster tf_broadcaster;
		/*const*/
		const int size_data = 6;	//wx, wy, wx, vx, vy, vz
		const double time_enough = 60.0;	//[s]
		const double threshold_deflection_imu_angular = 0.03;
		const double threshold_deflection_imu_linear = 0.2;
		/* const double threshold_abs_imu_linear = 0.2; */
		/* const double threshold_abs_imu_angular = 0.03; */
		/* const double threshold_abs_odom_linear = 1.0e-3; */
		/* const double threshold_abs_odom_angular = 1.0e-3; */
		const int min_record_size = 10;
		/*objects*/
		geometry_msgs::Quaternion initial_pose;
		std::vector<sensor_msgs::Imu> record;
		sensor_msgs::Imu average;
		/*flags*/
		bool imu_is_moving = false;
		bool initial_algnment_is_done = false;
	public:
		ImuInitialAlignment();
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		Eigen::VectorXd InputToVector(sensor_msgs::Imu imu);
		sensor_msgs::Imu InputToImuMsg(Eigen::VectorXd Vec);
		void ComputeAverage(void);
		bool JudgeMovingIMU(sensor_msgs::Imu imu);
		void ComputeInitialPose(void);
		void InputZero(void);
		void Publication(void);
};

ImuInitialAlignment::ImuInitialAlignment()
{
	sub_imu = nh.subscribe("/imu/data", 1, &ImuInitialAlignment::CallbackIMU, this);
	pub_inipose = nh.advertise<geometry_msgs::Quaternion>("/initial_pose", 1);
	pub_bias = nh.advertise<sensor_msgs::Imu>("/imu/bias", 1);
}

void ImuInitialAlignment::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	if(!initial_algnment_is_done){
		double duration = 0;
		bool imu_is_moving = false;
		if(record.size()>0){
			duration = (msg->header.stamp - record[0].header.stamp).toSec();
			imu_is_moving = JudgeMovingIMU(*msg);
		}

		if(imu_is_moving || duration>time_enough){
			initial_algnment_is_done = true;
			std::cout << "record.size() = " << record.size() << std::endl;
			if(duration>time_enough)	std::cout << "Duration is now enough!(" << duration << " [s])" << std::endl;
			if(record.size()<min_record_size){
				std::cout << ">> Initial alignment is FAILEiD" << std::endl;
				InputZero();
			}
			else{
				std::cout << ">> Initial alignment is DONE!" << std::endl;
				ComputeInitialPose();
			}
		}
		else{
			record.push_back(*msg);
			ComputeAverage();
		}
	}
	else	Publication();
}

void ImuInitialAlignment::ComputeAverage(void)
{
	Eigen::VectorXd Ave = Eigen::VectorXd::Zero(size_data);
	
	for(size_t i=0;i<record.size();i++)	Ave += InputToVector(record[i]);
	Ave /= (double)record.size();

	average = InputToImuMsg(Ave);
}

bool ImuInitialAlignment::JudgeMovingIMU(sensor_msgs::Imu imu)
{
	Eigen::VectorXd Ave = InputToVector(average);
	Eigen::VectorXd New = InputToVector(imu);

	for(int i=0;i<3;i++){
		double deflection = fabs(Ave(i) - New(i));
		if(deflection > threshold_deflection_imu_angular){
			std::cout << "Started moving!(imu_angular) index#: " << i << std::endl;
			return true;
		}
	}
	for(int i=3;i<6;i++){
		double deflection = fabs(Ave(i) - New(i));
		if(deflection > threshold_deflection_imu_linear){
			std::cout << "Started moving!(imu_linear) index#: " << i << std::endl;
			return true;
		}
	}

	return false;
}

Eigen::VectorXd ImuInitialAlignment::InputToVector(sensor_msgs::Imu imu)
{
	Eigen::VectorXd Vec(size_data);
	Vec <<	imu.angular_velocity.x,
			imu.angular_velocity.y,
			imu.angular_velocity.z,
			imu.linear_acceleration.x,
			imu.linear_acceleration.y,
			imu.linear_acceleration.z;
	return Vec;
}

sensor_msgs::Imu ImuInitialAlignment::InputToImuMsg(Eigen::VectorXd Vec)
{
	sensor_msgs::Imu imu;
	imu.angular_velocity.x = Vec(0);
	imu.angular_velocity.y = Vec(1);
	imu.angular_velocity.z = Vec(2);
	imu.linear_acceleration.x = Vec(3);
	imu.linear_acceleration.y = Vec(4);
	imu.linear_acceleration.z = Vec(5);
	return imu;
}

void ImuInitialAlignment::ComputeInitialPose(void)
{
	double ax = average.linear_acceleration.x;
	double ay = average.linear_acceleration.y;
	double az = average.linear_acceleration.z;
	double r = atan2(ay, az);
	double p = atan2(-ax, sqrt(ay*ay + az*az));
	quaternionTFToMsg(tf::createQuaternionFromRPY(r, p, 0), initial_pose);

	std::cout << "inittial (roll, pitch) = (" << r/M_PI*180.0 << ", " << p/M_PI*180.0 << ")[deg]" << std::endl;
}

void ImuInitialAlignment::InputZero(void)
{
	average = InputToImuMsg(Eigen::VectorXd::Zero(size_data));
	initial_pose.x = 0.0;
	initial_pose.y = 0.0;
	initial_pose.z = 0.0;
	initial_pose.w = 1.0;
}

void ImuInitialAlignment::Publication(void)
{
	/*publish*/
	pub_inipose.publish(initial_pose);
	pub_bias.publish(average);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = ros::Time::now();
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/initial_pose";
	transform.transform.translation.x = 0.0;
	transform.transform.translation.y = 0.0;
	transform.transform.translation.z = 0.0;
	transform.transform.rotation = initial_pose;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_initial_alignment");

	std::cout << "Initial Alignment START!" << std::endl;
	ImuInitialAlignment imu_initial_alignment;

	ros::spin();
}
