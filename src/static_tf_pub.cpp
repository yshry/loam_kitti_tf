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
        geometry_msgs::TransformStamped ccr, ccl, cgr, cgl, vl, il, bl;
        //, vcl;

        try{
            il = tfBuffer.lookupTransform("velo_link", "imu_link", ros::Time(0));
            bl = tfBuffer.lookupTransform("imu_link", "base_link", ros::Time(0));
            cgl = tfBuffer.lookupTransform("imu_link", "camera_gray_left", ros::Time(0));
            cgr = tfBuffer.lookupTransform("imu_link", "camera_gray_right", ros::Time(0));
            ccl = tfBuffer.lookupTransform("imu_link", "camera_color_left", ros::Time(0));
            ccr = tfBuffer.lookupTransform("imu_link", "camera_color_right", ros::Time(0));

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
		vl.child_frame_id ="loam_velo_link";
		vl.transform.translation.x = 0.0;
		vl.transform.translation.y = 0.0;
		vl.transform.translation.z = 0.0;

		vl.transform.rotation.x = quat.x();
		vl.transform.rotation.y = quat.y();
		vl.transform.rotation.z = quat.z();
		vl.transform.rotation.w = quat.w();

        il.header.frame_id = "loam_velo_link";
        il.child_frame_id = "loam_imu_link";
        bl.header.frame_id = "loam_imu_link";
        bl.child_frame_id = "loam_base_link";
        cgl.header.frame_id = "loam_imu_link";
        cgl.child_frame_id = "loam_camera_gray_left";
        cgr.header.frame_id = "loam_imu_link";
        cgr.child_frame_id = "loam_camera_gray_right";
        ccl.header.frame_id = "loam_imu_link";
        ccl.child_frame_id = "loam_camera_color_left";
        ccr.header.frame_id = "loam_imu_link";
        ccr.child_frame_id = "loam_camera_color_right";
        

		static tf2_ros::StaticTransformBroadcaster static_broadcaster;
		static_broadcaster.sendTransform(vl);
		static_broadcaster.sendTransform(il);
		static_broadcaster.sendTransform(bl);
		static_broadcaster.sendTransform(cgl);
		static_broadcaster.sendTransform(cgr);
		static_broadcaster.sendTransform(ccl);
		static_broadcaster.sendTransform(ccr);

        rate.sleep();
    }
    return 0;

}