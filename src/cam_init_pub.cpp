#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//inverse kitti transforms and broadcast.

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_pub");
    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(1.0);

    while (node.ok())
    {
        geometry_msgs::TransformStamped ii;
   		tf2::Quaternion quat;

		quat.setEuler(3.0*M_PI/2.0, 3.0*M_PI/2.0, 0.0);
        ros::Time stamp = ros::Time(0);
        
        ii.header.stamp = stamp;
        ii.header.frame_id = "camera_init";
        ii.child_frame_id = "loam_camera_init";

        ii.transform.translation.x = 0.0;
		ii.transform.translation.y = 0.0;
		ii.transform.translation.z = 0.0;

		ii.transform.rotation.x = quat.x();
		ii.transform.rotation.y = quat.y();
		ii.transform.rotation.z = quat.z();
		ii.transform.rotation.w = quat.w();
        
        rate.sleep();
    }
    return 0;

}