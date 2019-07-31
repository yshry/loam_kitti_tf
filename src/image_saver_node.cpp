#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

class ImageSaver
{
    public:
    ImageSaver(ros::NodeHandle nh, const std::string& prefix, const std::string& dir, const std::string& topic)
    : __it(nh), __prefix(prefix), __dir(dir)
    {
        this->__sub_img = this->__it.subscribe(topic,10, &ImageSaver::__imageCallback, this);
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
    image_transport::ImageTransport __it;
    image_transport::Subscriber __sub_img;
    std::string __prefix;
    std::string __dir;
    //std::string __encoding;

    void __imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        std::stringstream ss;
        ss << this->__dir << "/";
        ss << boost::format("%|010|")%msg->header.stamp.sec << "_";
        ss << boost::format("%|09|")%msg->header.stamp.nsec << "_";
        ss << this->__prefix <<".dat";
        std::ofstream ofs(ss.str().c_str(), std::ofstream::out | std::ofstream::binary);
        uint32_t msg_size = msg->step * msg->height;
        ofs.write(reinterpret_cast<const char*>(&msg->data[0]), msg_size);
        ofs.close();
        ROS_INFO ("output %s" ,ss.str().c_str() );
    }

};


int main (int argc, char** argv)
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed Options");
    desc.add_options()
        ("help", "produce help message")
        ("prefix,P", po::value<std::string>(), "prefix for output filename")
        ("dir,F", po::value<std::string>(), "output directory path")
        ("date,D", po::bool_switch(), "set to add datetime to directory name")
        ("topic,T", po::value<std::string>(), "topic name to subscribe")
        ("node,N", po::value<std::string>()->default_value("image_saver"), "node name")
    ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

    po::notify (vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }

    //std::cout << vm["dir_path"].as<std::string>() << ", " << vm["date_flag"].as<bool>() << std::endl;

    std::string prefix = vm["prefix"].as<std::string>();
    std::string dir_path = vm["dir"].as<std::string>();
    std::string topic = vm["topic"].as<std::string>();
    std::string node_name = vm["node"].as<std::string>();
    bool date_flag = ! vm["date"].as<bool>();
    
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%4Y%2m%2d_%2H%2M%2S", timeinfo);

    if (date_flag)
        dir_path = dir_path + "_"+buffer;

    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    if(boost::filesystem::create_directory(dir_path))
    {
        ROS_INFO( "directory \"%s\" created", dir_path.c_str());
    }

    ImageSaver is(nh, prefix, dir_path, topic);
    is.spin();

    return 0;
}