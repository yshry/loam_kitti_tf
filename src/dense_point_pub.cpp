#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/transforms.h>
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
        this->__image_depth_sub = this->__it.subscribe("/loam/camera_color_left/image_depth", 10, &DensePointPub::__imageDepthCallback, this);
        this->__image_raw_sub = this->__it.subscribe("/loam/camera_color_left/image_raw", 10, &DensePointPub::__imageRawCallback, this);
        //this->__cam_info_sub = nh.subscribe("/loam/camera_color_left/camera_info", 10, &DensePointPub::__camInfoCallback, this);
        this->__point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/dense_point", 10);

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
            this->__process();
			rate.sleep();
		}
	}

    private:
    //ros::NodeHandle __nh;
    image_transport::ImageTransport __it;
    image_transport::Subscriber __image_depth_sub;
    image_transport::Subscriber __image_raw_sub;
    ros::Subscriber __cam_info_sub;
    ros::Publisher __point_cloud_pub;
    Eigen::Matrix3f __intrinsics;
    Eigen::Matrix3f __inv_intrinsics;
    std::string __frame_id;
    //tf2_ros::Buffer __tfBuffer;
    //tf::TransformListener __tfListener;
    cv::Mat __image_depth;
    cv::Mat __image_raw;
    ros::Time __image_depth_stamp;
    ros::Time __image_raw_stamp;
    bool __new_depth = false;
    bool __new_raw = false;

    void __imageDepthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        this->__image_depth = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16)->image;
        this->__image_depth_stamp = msg->header.stamp;
        this->__new_depth = true;
        this->__frame_id = msg->header.frame_id;
    }

    void __imageRawCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        this->__image_raw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        this->__image_raw_stamp = msg->header.stamp;
        this->__new_raw = true;
    }

    void __process()
    {
        if (!(this->__new_depth && this->__new_raw))
            return;
        
        ROS_INFO("new depth and new raw");


        if (fabs((this->__image_depth_stamp - this->__image_raw_stamp).toSec()) < 0.05)
            return;
        ROS_INFO("stamp matched");
        
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        int num_pixels = this->__image_depth.rows * this->__image_depth.cols;
        cloud.width = num_pixels;
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize(num_pixels);

        int num_points = 0;

        for (int y=0; y<this->__image_depth.rows; y++)
        {
            for (int x=0; x<this->__image_depth.cols; x++)
            {
                uint16_t depth = this->__image_depth.at<uint16_t>(y,x);
                cv::Vec3b color = this->__image_raw.at<cv::Vec3b>(y,x);
                //if (depth >0 && depth < 10*256)
                if (depth >0)
                {
                    Eigen::Vector3f p = (float)(depth / 256.0) * this->__inv_intrinsics * Eigen::Vector3f(x,y,1);
                    cloud[num_points].x = p(0);
                    cloud[num_points].y = p(1);
                    cloud[num_points].z = p(2);
                    cloud[num_points].r = color(2);
                    cloud[num_points].g = color(1);
                    cloud[num_points].b = color(0);
                    num_points++;
                }
               
            }
        }
        cloud.points.resize(num_points);
        cloud.width = num_points;

        geometry_msgs::TransformStamped transform_stamped;
        sensor_msgs::PointCloud2 point_msg;
        pcl::toROSMsg(cloud, point_msg);
        point_msg.header.stamp = this->__image_depth_stamp;
        point_msg.header.frame_id = this->__frame_id;
 
        this->__point_cloud_pub.publish(point_msg);
        this->__new_depth = false;
        this->__new_raw = false;
    }

    void __imageCallbackOld(const sensor_msgs::ImageConstPtr& msg)
    {
        /*
        const cv::Mat casted_data = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16)->image;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        int num_pixels = casted_data.rows * casted_data.cols;
        cloud.width = num_pixels;
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize(num_pixels);

        int num_points = 0;

        for (int y=0; y<casted_data.rows; y++)
        {
            for (int x=0; x<casted_data.cols; x++)
            {
                uint16_t depth = casted_data.at<uint16_t>(y,x);
                //if (depth >0 && depth < 10*256)
                if (depth >0)
                {
                    Eigen::Vector3f p = (float)(depth / 256.0) * this->__inv_intrinsics * Eigen::Vector3f(x,y,1);
                    cloud[num_points].x = p(0);
                    cloud[num_points].y = p(1);
                    cloud[num_points].z = p(2);
                    num_points++;
                }
               
            }
        }
        cloud.points.resize(num_points);
        cloud.width = num_points;

        geometry_msgs::TransformStamped transform_stamped;
        sensor_msgs::PointCloud2 point_msg;
        pcl::toROSMsg(cloud, point_msg);
        point_msg.header.stamp = msg->header.stamp;
        point_msg.header.frame_id = msg->header.frame_id;
 
        this->__point_cloud_pub.publish(point_msg);
        */
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

    return 0;
}