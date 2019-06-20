#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class DensePointPub
{
    public:
    DensePointPub(ros::NodeHandle& nh):__it(nh), __tfListener(__tfBuffer)
    {
        //this->__nh = nh;
        this->__image_sub = this->__it.subscribe("/loam/camera_color_left/image_depth", 10, &DensePointPub::__imageCallback, this);
        this->__cam_info_sub = nh.subscribe("/loam/camera_color_left/camera_info", 10, &DensePointPub::__camInfoCallback, this);
        this->__point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/dense_depth", 10);
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
    tf2_ros::Buffer __tfBuffer;
    tf2_ros::TransformListener __tfListener;



    void __imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //std::vector<uint16_t> casted_data (msg->data.begin(), msg->data.end());
        //const uint16_t* casted_data = reinterpret_cast<const uint16_t*> (msg->data.data());

        const cv::Mat casted_data = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16)->image;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        int num_pixels = casted_data.rows * casted_data.cols;
        cloud.width = num_pixels;
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.points.resize(num_pixels);

        //std::cout << num_pixels <<", " << msg->width * msg->height << std::endl;

        int num_points = 0;

        for (int y=0; y<casted_data.rows; y++)
        {
            for (int x=0; x<casted_data.cols; x++)
            {
                uint16_t depth = casted_data.at<uint16_t>(y,x);
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

        /*

        geometry_msgs::TransformStamped point_transform_stamped;
        //tf::Transform point_transform;
        try{
            point_transform_stamped = this->__tfBuffer.lookupTransform("loam_camera_color_left", "camera_init", msg->header.stamp);
            //point_transform = point_transform_stamped.transform;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }

        pcl_ros::transformPointCloud(cloud, cloud_out, point_transform_stamped.transform);
        */

        sensor_msgs::PointCloud2 point_msg;
        pcl::toROSMsg(cloud, point_msg);

        point_msg.header.stamp = msg->header.stamp;
        //point_msg.header.stamp = ros::Time::now();
        //point_msg.header.frame_id = this->__frame_id;
        point_msg.header.frame_id = msg->header.frame_id;

        this->__point_cloud_pub.publish(point_msg);


    }
    
    void __camInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        for (int i=0; i<9; i++)
        {
            int x = i %3;
            int y = i /3;
            this->__intrinsics(y,x)= msg->K[i];
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