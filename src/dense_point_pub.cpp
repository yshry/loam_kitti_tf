#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
//#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
//#include <tf2_sensor_msgs.h>


class DensePointPub
{
    public:
    DensePointPub(ros::NodeHandle& nh):__it(nh)
    {
        //this->__nh = nh;
        this->__image_sub = this->__it.subscribe("/loam/camera_color_left/image_depth", 10, &DensePointPub::__imageCallback, this);
        //this->__cam_info_sub = nh.subscribe("/loam/camera_color_left/camera_info", 10, &DensePointPub::__camInfoCallback, this);
        this->__point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/dense_depth", 10);

        sensor_msgs::CameraInfoConstPtr caminfo_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/loam/camera_color_left/camera_info");
        this->__camInfoCallback(caminfo_msg);
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
    //ros::NodeHandle __nh;
    image_transport::ImageTransport __it;
    image_transport::Subscriber __image_sub;
    ros::Subscriber __cam_info_sub;
    ros::Publisher __point_cloud_pub;
    Eigen::Matrix3f __intrinsics;
    Eigen::Matrix3f __inv_intrinsics;
    const std::string __frame_id = "/camera_init";
    //tf2_ros::Buffer __tfBuffer;
    tf::TransformListener __tfListener;



    void __imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //std::vector<uint16_t> casted_data (msg->data.begin(), msg->data.end());
        //const uint16_t* casted_data = reinterpret_cast<const uint16_t*> (msg->data.data());

        const cv::Mat casted_data = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16)->image;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        //pcl::PointCloud<pcl::PointXYZ> cloud_out;
        int num_pixels = casted_data.rows * casted_data.cols;
        cloud.width = num_pixels;
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize(num_pixels);

        //std::cout << num_pixels <<", " << msg->width * msg->height << std::endl;

        int num_points = 0;

        sensor_msgs::ImageConstPtr color_img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/loam/camera_color_left/image_raw");


        for (int y=0; y<casted_data.rows; y++)
        {
            for (int x=0; x<casted_data.cols; x++)
            {
                uint16_t depth = casted_data.at<uint16_t>(y,x);
                //if (depth >0 && depth < 10*256)
                if (depth >0)
                {
                    Eigen::Vector3f p = (float)(depth / 256.0) * this->__inv_intrinsics * Eigen::Vector3f(x,y,1);
                    //Eigen::Vector3f p = this->__inv_intrinsics * Eigen::Vector3f(x,y,1);
                    cloud[num_points].x = p(0);
                    cloud[num_points].y = p(1);
                    cloud[num_points].z = p(2);
                    //cloud[num_points].x = ((float)x - this->__intrinsics(0,2))/ this->__intrinsics(0,0);
                    //cloud[num_points].y = ((float)y  - this->__intrinsics(1,2)) / this->__intrinsics(1,1);
                    //cloud[num_points].z = 1.0;
                    num_points++;
                }
               
            }
        }
        cloud.points.resize(num_points);
        cloud.width = num_points;

        //tf2::Stamped<tf2::Transform> point_transform_stamped;
        //tf::Transform point_transform;
        //tf::Transform point_transform_stamped;
        geometry_msgs::TransformStamped transform_stamped;
        //try{
            //transform_stamped = this->__tfBuffer.lookupTransform("loam_camera_color_left", "camera_init", msg->header.stamp);
            //tf2::fromMsg(tmp_pt, point_transform_stamped);
            //point_transform = point_transform_stamped.transform;
        //}
        //catch (tf2::TransformException &ex)
        //{
            //ROS_WARN("%s",ex.what());
            //return;
        //}

        //pcl_ros::transformPointCloud("/camera_init", point_transform_stamped, cloud, cloud_out);
        //pcl_ros::transformPointCloud(cloud, cloud_out, point_transform_stamped);

        sensor_msgs::PointCloud2 point_msg, point_msg_in;
        pcl::toROSMsg(cloud, point_msg_in);
        point_msg_in.header.stamp = msg->header.stamp;
        point_msg_in.header.frame_id = msg->header.frame_id;
        //tf2::doTransform(point_msg_in, point_msg, transform_stamped);
        std::string target_frame = "camera_init";
        //ros::Time target_time = msg->header.stamp;
        //std::string fixed_frame = msg->header.frame_id;
        __tfListener.waitForTransform(target_frame, point_msg_in.header.frame_id, point_msg_in.header.stamp, ros::Duration(1.0));

        //pcl_ros::transformPointCloud(target_frame, target_time, cloud, fixed_frame, cloud_out, this->__tfListener )
        pcl_ros::transformPointCloud(target_frame, point_msg_in, point_msg, this->__tfListener);

        //pcl::toROSMsg(cloud_out, point_msg);
        //point_msg.header.stamp = msg->header.stamp;
        //point_msg.header.stamp = ros::Time::now();
        //point_msg.header.frame_id = this->__frame_id;
        //point_msg.header.frame_id = "/camera_init";
 
        this->__point_cloud_pub.publish(point_msg);


    }
    
    void __camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        for (int i=0; i<12; i++)
        {
            int x = i %4;
            int y = i /4;
            if (x==3 || y==3) continue;

            this->__intrinsics(y,x)= msg->P[i];
        }   
        //std::cout << this->__intrinsics << std::endl;
        this->__inv_intrinsics = this->__intrinsics.inverse();
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dense_point_pub");
    ros::NodeHandle nh;
    DensePointPub dense_point_pub(nh);
    dense_point_pub.spin();

}