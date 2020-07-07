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
    DensePointSaver(ros::NodeHandle& nh, const std::string& dir_path, const std::string& msg_name, const int& stack, bool compressed)
    : __cloud(new pcl::PCLPointCloud2()), __pcd_writer(), __stack_size(stack), __compressed(compressed), __timeout(stack/10*2)
    {
        this->__dir_path = boost::filesystem::path(dir_path);
        if(boost::filesystem::create_directory(this->__dir_path))
        {
            ROS_INFO( (boost::format("directory \"%s\" created") % this->__dir_path).str().c_str());
        }
        ROS_INFO ((boost::format("set output directory to \"%s\"") % this->__dir_path).str().c_str());
        this->__dense_point_subscriber = nh.subscribe<sensor_msgs::PointCloud2>(msg_name, 1000, &DensePointSaver::__denseDepthCallback, this);
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
    const float __timeout;
    const int __stack_size;
    unsigned int __current_stack = 0;
    pcl::PCLPointCloud2::Ptr __cloud;
    tf::TransformListener __tfListener;
    std::string __target_frame = "camera_init";
    pcl::PCDWriter __pcd_writer;
    ros::Time __last_msg_time;
    bool __compressed;
    std::vector<ros::Time> __time_vec;

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
};


int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "produce help message")
        ("dir_path", po::value<std::string>()->default_value("./dense_point"), "output directory path")
        ("date_flag,D", po::bool_switch(), "set to add datetime to directory name") 
        ("message,M", po::value<std::string>()->default_value("/dense_point"), "pointcloud message to subscribe")
        ("stack,S", po::value<int>()->default_value(10), "number to stack")
        ("raw,R", po::bool_switch(), "set to save uncompressed pcd (default compressed)") 
        ("id,I", po::value<int>()->default_value(0), "id")
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
    bool date_flag = vm["date_flag"].as<bool>();
    std::string msg_name = vm["message"].as<std::string>();
    int stack = vm["stack"].as<int>();
    bool compressed = ! vm["raw"].as<bool>();
    int id = vm["id"].as<int>();

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%4Y%2m%2d_%2H%2M%2S", timeinfo);

    if (date_flag)
        dir_path = dir_path + "_"+buffer;
    //std::cout << dir_path;

    std::stringstream ss;
    ss << "dense_point_saver_" << id;

    ros::init(argc, argv, ss.str());
    ros::NodeHandle nh;

    DensePointSaver dense_point_saver(nh, dir_path, msg_name, stack, compressed);
    dense_point_saver.spin();

    return 0;
}