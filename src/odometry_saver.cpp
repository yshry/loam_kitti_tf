#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <fstream>

class OdometrySaver
{
    public:
    OdometrySaver(ros::NodeHandle& nh, const std::string& dir_path)
    :__dir_path(boost::filesystem::path(dir_path).string()), __current_file(0), __current_line(0)
    {
        this->__sub_odom = nh.subscribe<nav_msgs::Odometry>("/integrated_to_init", 10, &OdometrySaver::__odomCallback, this);
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
    ros::Subscriber __sub_odom;
    std::string __dir_path;
    int __current_line;
    int __current_file;

    void __odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        std::stringstream ss;
        ss << this->__dir_path << "/";
        ss << boost::format("%|010|")%__current_file << ".txt";
        std::ofstream ofs;
        if (this->__current_line == 0)
        {
            ofs.open(ss.str(), std::ofstream::out);
        }
        else
        {
            ofs.open(ss.str(), std::ofstream::out | std::ofstream::app);            
        }
        ofs << msg->pose.pose.position.x << ", " << msg->pose.pose.position.y << ", " << msg->pose.pose.position.z <<", ";
        ofs << msg->pose.pose.orientation.w << ", " << msg->pose.pose.orientation.x << ", " << msg->pose.pose.orientation.y <<", " << msg->pose.pose.orientation.z << ", ";
        ofs << msg->header.stamp.sec <<", " << msg->header.stamp.nsec << std::endl;
        ofs.close();

        this->__current_line ++;
        if (this->__current_line > 1e106)
        {
            this->__current_file ++;
            this->__current_line = 0;
        }

    }
};


int main(int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "produce help message")
        ("dir_path", po::value<std::string>()->default_value("./odometry"), "output directory path")
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

    if(boost::filesystem::create_directory(dir_path))
    {
        ROS_INFO( "directory \"%s\" created", dir_path.c_str());
    }


    OdometrySaver os(nh, dir_path);
    os.spin();

    return 0;
}