#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>


//camera info
//make it to class

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher& pub, const std::string& frame_id)
{
    sensor_msgs::Image new_msg = *msg;
    new_msg.header.frame_id=frame_id;
    //new_msg.header.stamp = msg->header.stamp;
    pub.publish(new_msg);
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg, const ros::Publisher& pub, const std::string& frame_id)
{
    sensor_msgs::CameraInfo new_msg = *msg;
    new_msg.header.frame_id = frame_id;
    std::fill(new_msg.D.begin(), new_msg.D.end(), 0.0); 
    pub.publish(new_msg);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "image_trans");
    ros::NodeHandle nh;
    image_transport::ImageTransport it (nh);
    
    image_transport::Publisher pub_cclid = it.advertise("/loam/camera_color_left/image_depth", 10);
    image_transport::Subscriber sub_cclid = it.subscribe("/kitti/camera_color_left/image_depth",10, 
        boost::bind(&imageCallback, _1, pub_cclid, "loam_camera_color_left") 
        );

    image_transport::Publisher pub_cclir = it.advertise("/loam/camera_color_left/image_raw", 10);
    image_transport::Subscriber sub_cclir = it.subscribe("/kitti/camera_color_left/image_raw",10, 
        boost::bind(&imageCallback, _1, pub_cclir, "loam_camera_color_left") 
        );

    ros::Publisher pub_cclif = nh.advertise<sensor_msgs::CameraInfo>("/loam/camera_color_left/camera_info", 10);
    ros::Subscriber sub_cclif = nh.subscribe<sensor_msgs::CameraInfo>("/kitti/camera_color_left/camera_info", 10,
        boost::bind(&cameraInfoCallback, _1, pub_cclif, "loam_camera_color_left")
        );


    ros::spin();
}
