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
#include <boost/program_options.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.h>
#include <ctime>
#include <thread>

class DensePointSaver
{
    public:
    DensePointSaver(ros::NodeHandle nh, const std::string dir_path)
    : __cloud(new pcl::PCLPointCloud2()), __pcd_writer()
    {
        this->__dir_path = boost::filesystem::path(dir_path);
        if(boost::filesystem::create_directory(this->__dir_path))
        {
            ROS_INFO( (boost::format("directory \"%s\" created") % this->__dir_path).str().c_str());
        }
        ROS_INFO ((boost::format("set output directory to \"%s\"") % this->__dir_path).str().c_str());
        this->__dense_point_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/dense_point", 10, &DensePointSaver::__denseDepthCallback, this);
        //this->__cloud.reset(new pcl::PCLPointCloud2());
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
    ros::Subscriber __dense_point_subscriber;
    unsigned int __file_index=0;
    const float __timeout = 5.0;
    const int __stack_size = 10;
    unsigned int __current_stack = 0;
    pcl::PCLPointCloud2::Ptr __cloud;
    tf::TransformListener __tfListener;
    std::string __target_frame = "camera_init";
    pcl::PCDWriter __pcd_writer;
    ros::Time __last_msg_time;

    void __denseDepthCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        pcl::PCLPointCloud2 msg_cloud;
        sensor_msgs::PointCloud2 msg_out;

        __tfListener.waitForTransform(this->__target_frame, msg->header.frame_id, msg->header.stamp, ros::Duration(1.0));
        pcl_ros::transformPointCloud(this->__target_frame, *msg, msg_out, this->__tfListener);


        pcl_conversions::toPCL(msg_out, msg_cloud);
        pcl::concatenatePointCloud (*this->__cloud, msg_cloud, *this->__cloud);
        this->__current_stack += 1;
        this->__last_msg_time = msg->header.stamp;
    }

    void __savePointCloud()
    {
        if (this->__current_stack == 0)
            return;
        if (this->__current_stack < this->__stack_size || 
            fabs((ros::Time::now() - this->__last_msg_time).toSec()) < this->__timeout)
            return;
        
        std::string filename = (this->__dir_path / (boost::format("%010d.pcd")%this->__file_index).str()).string();
        this->__pcd_writer.writeBinaryCompressed(filename, *this->__cloud);
        this->__current_stack = 0;
        this->__file_index += 1;
        this->__cloud.reset(new pcl::PCLPointCloud2());
    }
};


int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "produce help message")
        ("dir_path", po::value<std::string>()->default_value("./dense_point"), "output directory path")
        ("date_flag,D", po::bool_switch(), "set to add datetime to directory name") 
    ;

    po::positional_options_description p;
    p.add("dir_path", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);

    po::notify (vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    //std::cout << vm["dir_path"].as<std::string>() << ", " << vm["date_flag"].as<bool>() << std::endl;

    std::string dir_path = vm["dir_path"].as<std::string>();
    bool date_flag = ! vm["date_flag"].as<bool>();

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%4Y%2m%2d_%2H%2M%2S", timeinfo);

    if (date_flag)
        dir_path = dir_path + "_"+buffer;
    //std::cout << dir_path;

    ros::init(argc, argv, "dense_point_saver");
    ros::NodeHandle nh;

    DensePointSaver dense_point_saver(nh, dir_path);
    dense_point_saver.spin();

    return 0;
}