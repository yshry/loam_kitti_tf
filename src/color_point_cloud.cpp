#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/io.h>
#include <pcl/common/concatenate.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
//#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/transforms.h>
#include <ctime>
#include <thread>

class ColorPointCloud
{
    public:
    ColorPointCloud(ros::NodeHandle nh)
    :__it(nh), __new_raw(false), __new_point(false)
    {
        this->__point_sub = nh.subscribe<sensor_msgs::PointCloud2>("/pointcloud", 10, &ColorPointCloud::__pointCallback, this);
        this->__image_sub = __it.subscribe("/image_raw", 10, &ColorPointCloud::__imageRawCallback, this);
        sensor_msgs::CameraInfoConstPtr caminfo_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera_info");
        this->__camInfoCallback(caminfo_msg);

        this->__point_pub = nh.advertise<sensor_msgs::PointCloud2>("/color_points", 10);
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
    image_transport::ImageTransport __it;
    image_transport::Subscriber __image_sub;
    ros::Subscriber __point_sub;
    ros::Publisher __point_pub;

    //pcl::PCLPointCloud2::Ptr __cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr __cloud;

    //sensor_msgs::PointCloud2ConstPtr __cloud_msg;
    tf::TransformListener __tfListener;
    ros::Time __cloud_stamp;
    cv::Mat __image_raw;
    ros::Time __image_raw_stamp;
    std::string __image_raw_frame;
    //std::string __pointcloud_frame;
    bool __new_raw, __new_point;


    Eigen::Matrix3f __intrinsics;
    //Eigen::Matrix3f __inv_intrinsics;


    void __pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        sensor_msgs::PointCloud2 msg_trans;        
        __tfListener.waitForTransform(this->__image_raw_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(this->__image_raw_frame, *msg, msg_trans, this->__tfListener);
        pcl::PCLPointCloud2 tmp_cloud;
        pcl_conversions::toPCL(msg_trans, tmp_cloud);

        __cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromPCLPointCloud2(tmp_cloud, *__cloud);

        this->__new_point = true;
        this->__cloud_stamp = msg->header.stamp;

        ROS_INFO("received point cloud");
    }

    void __imageRawCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        this->__image_raw = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;
        this->__image_raw_stamp = msg->header.stamp;
        this->__new_raw = true;

        ROS_INFO("received image");
    }

    void __camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        for (int i=0; i<12; i++)
        {
            int x = i %4;
            int y = i /4;
            if (x==3 || y==3) continue;

            this->__intrinsics(y,x)= msg->P[i];
            this->__image_raw_frame = msg->header.frame_id;
        }   
        //std::cout << this->__intrinsics << std::endl;
        //this->__inv_intrinsics = this->__intrinsics.inverse();
    }

    void __process()
    {
        if (fabs((this->__image_raw_stamp - this->__cloud_stamp).toSec()) < 0.05)
            return;
        
        if (!(this->__new_raw && this->__new_point))
            return;
        
        ROS_INFO("internal process");

        try
        {
            this->__internal_process();
        }
        catch( tf::TransformException ex)
        {
            ROS_ERROR("transfrom exception : %s",ex.what());
        }
    }

    inline void __internal_process()
    {

        pcl::PointCloud<pcl::PointXYZRGB> cloud2;

        pcl::PointCloud<pcl::PointXYZRGB>::iterator it = this->__cloud->begin();
        
        while (it != this->__cloud->end())
        {
            Eigen::Vector3f p = this->__intrinsics * Eigen::Vector3f(it->x, it->y, it->z);
            int p_x = std::round(p(0) / p(2));
            int p_y = std::round(p(1) / p(2));

            if (p(2) > 0 && p_x >0 && p_x < this->__image_raw.cols && p_y >0 && p_y < __image_raw.rows)
            {                
                cv::Vec3b color = this->__image_raw.at<cv::Vec3b>(p_y, p_x);

                pcl::PointXYZRGB cp;
                cp.x = it->x;
                cp.y = it->y;
                cp.z = it->z;
                cp.r = color(0);
                cp.g = color(1);
                cp.b = color(2);
                cloud2.push_back(cp);
            }
            it++;
        }

        sensor_msgs::PointCloud2 point_msg;
        pcl::toROSMsg(cloud2, point_msg);
        point_msg.header.stamp = this->__image_raw_stamp;
        point_msg.header.frame_id = this->__image_raw_frame;
 
        this->__point_pub.publish(point_msg);
        this->__new_raw = false;
        this->__new_point = false;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "color_point_cloud");
    ros::NodeHandle nh;

    ColorPointCloud cp(nh);
    cp.spin();

    return 0;
}