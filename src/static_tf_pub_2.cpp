#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

//inverse kitti transforms and broadcast.

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_tf_pub_2");
    ros::NodeHandle node;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(1.0);

    while (node.ok())
    {
        geometry_msgs::TransformStamped vl, il;
        //, vcl;

        try{
            il = tfBuffer.lookupTransform("velo_link", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

   		tf2::Quaternion quat;

		quat.setEuler(3.0*M_PI/2.0, 3.0*M_PI/2.0, 0.0);
        ros::Time stamp = il.header.stamp;

		vl.header.stamp = stamp;
		vl.header.frame_id ="camera";
		vl.child_frame_id ="loam_link";
		vl.transform.translation.x = 0.0;
		vl.transform.translation.y = 0.0;
		vl.transform.translation.z = 0.0;

		vl.transform.rotation.x = quat.x();
		vl.transform.rotation.y = quat.y();
		vl.transform.rotation.z = quat.z();
		vl.transform.rotation.w = quat.w();

        il.header.frame_id = "loam_link";
        il.child_frame_id = "base_link";

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(vl);
		static_broadcaster.sendTransform(il);


        rate.sleep();
    }
    return 0;

}