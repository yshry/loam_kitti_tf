#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/io.h>
#include <pcl/common/concatenate.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <ctime>
#include <thread>

class PointSaver
{
    public:
    PointSaver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : __cloud(new pcl::PCLPointCloud2()), __pcd_writer(), __stack_size(1), __compressed(false), __timeout(1.0)
    {
        this->setup(nh, pnh);
    }

    void setup(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        std::string sParam;
        int iParam;

        if (pnh.getParam("directory", sParam))
        {
            this->__dir_path = sParam;
        }
        else
        {
            this->__dir_path = "./point_saver";
        }

        if (pnh.getParam("msg", sParam))
        {
            this->__point_subscriber = nh.subscribe<sensor_msgs::PointCloud2>(sParam, 1000, &PointSaver::__pointCallback, this);
            ROS_INFO("Subscribed %s.", sParam);
        }
        else
        {
            this->__point_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 1000, &PointSaver::__pointCallback, this);            
        }

        if (pnh.getParam("compressed", iParam))
        {
            if (iParam) this->__compressed = true;
        }

        ROS_INFO("Set pcd format %d.", this->__compressed);


        if (pnh.getParam("stack", iParam))
        {
            if (iParam) this->__stack_size = iParam;
        }
        ROS_INFO("Set stack_size %d.", this->__stack_size);

        if(boost::filesystem::create_directory(this->__dir_path))
        {
            ROS_INFO( (boost::format("directory \"%s\" created") % this->__dir_path).str().c_str());
        }
        ROS_INFO ((boost::format("set output directory to \"%s\"") % this->__dir_path).str().c_str());

    }

    void spin()
    {
        ros::Rate rate(100);
		bool status = ros::ok();
		while (status)
		{
			ros::spinOnce();
			status = ros::ok();
            this->__savePointCloud();
			rate.sleep();
		}
    }

    private:
    boost::filesystem::path __dir_path;
    ros::Subscriber __point_subscriber;
    unsigned int __file_index=0;
    const float __timeout;
    int __stack_size;
    unsigned int __current_stack = 0;
    pcl::PCLPointCloud2::Ptr __cloud;
    tf::TransformListener __tfListener;
    std::string __target_frame = "camera_init";
    pcl::PCDWriter __pcd_writer;
    ros::Time __last_msg_time;
    bool __compressed;
    std::vector<ros::Time> __time_vec;

    void __pointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PCLPointCloud2 msg_cloud;
        sensor_msgs::PointCloud2 msg_out;

        __tfListener.waitForTransform(this->__target_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(this->__target_frame, *msg, msg_out, this->__tfListener);


        pcl_conversions::toPCL(msg_out, msg_cloud);
        pcl::concatenatePointCloud (*this->__cloud, msg_cloud, *this->__cloud);
        this->__current_stack += 1;
        this->__last_msg_time = msg->header.stamp;
        __time_vec.push_back(msg->header.stamp);
    }

    void __savePointCloud()
    {
        if (this->__current_stack == 0)
            return;
        if (this->__current_stack < this->__stack_size || 
            fabs((ros::Time::now() - this->__last_msg_time).toSec()) < this->__timeout)
            return;
        
        if (this->__compressed)
        {
            std::string filename = (this->__dir_path / (boost::format("%016d.pcd")%this->__file_index).str()).string();
            this->__pcd_writer.writeBinaryCompressed(filename, *this->__cloud);
        }
        else
        {
            std::string filename = (this->__dir_path / (boost::format("%016d.txt")%this->__file_index).str()).string();
            this->__pcd_writer.writeBinary(filename, *this->__cloud);
            //pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
            //pcl::fromPCLPointCloud2( *this->__cloud, laserCloudIn);
            //size_t cloudSize = laserCloudIn.size();
            //std::fstream ofs;
            //ofs.open(filename, std::ofstream::out);
            //for (int i = 0; i < cloudSize; i++) 
            //{
            //    ofs << laserCloudIn[i].x << " " << laserCloudIn[i].y << " " << laserCloudIn[i].z << " ";
            //    ofs << laserCloudIn[i].intensity << " " << "0 " << __time_vec[i].sec << ".";
            //    ofs << boost::format("%|09|")%__time_vec[i].nsec;
            //    ofs << std::endl;
            //}
            //ofs.close();

        }
        this->__current_stack = 0;
        this->__file_index += 1;
        this->__cloud.reset(new pcl::PCLPointCloud2());
        this->__time_vec.clear();
    }

    void __saveTransform()
    {
        
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointSaver");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PointSaver point_saver(nh, pnh);
    point_saver.spin();

    return 0;
}