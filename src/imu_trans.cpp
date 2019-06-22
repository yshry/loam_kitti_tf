#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
//#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_sensor_msgs.h>
#include <cmath>

class ImuTrans
{

    public:
	ImuTrans(): __tfListener(__tfBuffer){}

	int setup(ros::NodeHandle& node)
	{
		__imu_pub = node.advertise<sensor_msgs::Imu>("/imu/data", 100);
		__imu_sub = node.subscribe("/kitti/oxts/imu", 100, &ImuTrans::imuCallback, this);
	}
	
	void spin()
	{
		ros::Rate rate(100);
		bool status = ros::ok();
		while (status)
		{
			ros::spinOnce();
			status = ros::ok();
			rate.sleep();
		}
	}

	private:
	ros::Publisher __imu_pub;
	ros::Subscriber __imu_sub;
	//ros::Subscriber __tf_sub;
	tf2_ros::Buffer __tfBuffer;
    tf2_ros::TransformListener __tfListener;


	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_in)
	{
		try{
			geometry_msgs::TransformStamped ic = __tfBuffer.lookupTransform("loam_imu_link", "camera", ros::Time(0));
			geometry_msgs::TransformStamped static_transformStamped;
			static_transformStamped.transform.translation.x = 0.0;
			static_transformStamped.transform.translation.y = 0.0;
			static_transformStamped.transform.translation.z = 0.0;

			static_transformStamped.transform.rotation = ic.transform.rotation;

			sensor_msgs::Imu imu_out;
			imu_out.header.stamp = imu_in->header.stamp;
			tf2::doTransform(*imu_in, imu_out, static_transformStamped);

			__imu_pub.publish(imu_out);
		}
		catch(...)
		{
			ROS_WARN("imu_trans: Exception Occured in");
		}
	}

};

int main (int argc, char** argv)
{
	ros::init(argc, argv, "imu_trans");
	ros::NodeHandle node;
	ImuTrans imu_trans;

	if (imu_trans.setup(node))
	{
		imu_trans.spin();
	}

	return 0;
}
