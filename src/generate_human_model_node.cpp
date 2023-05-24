#include <xsens_awinda_ros/XSensClient.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/algorithm/string/replace.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_human_model");

    ros::NodeHandle nh("~");

    // Data parser initialization
    std::string model_name;
    nh.param<std::string>("model_name", model_name, "skeleton");
    int xsens_udp_port;
    nh.param<int>("udp_port", xsens_udp_port, 8001);

    boost::shared_ptr<XSensClient> xsens_client_ptr;
    try
    {
        xsens_client_ptr = boost::make_shared<XSensClient>(xsens_udp_port);
    }
    catch(const std::exception& err)
    {
        ROS_ERROR_STREAM(err.what());
        return -1;
    }
    
    if(!xsens_client_ptr->init())
    {
        ROS_ERROR_STREAM("XSens client initialization failed.");
        return -1;
    }

    // Get current time
    time_t curr_time = std::time(NULL);
    std::string time_str(std::ctime(&curr_time));
    boost::replace_all(time_str, " ", "_");
    boost::replace_all(time_str, ":", "_");

    std::string xsens_package_path = ros::package::getPath("hrii_xsens");
    std::string filename = xsens_package_path+"/config/model_"+time_str+".xacro";

    // Open file and write urdf
    std::ofstream file;
    file.open(filename);
    file << xsens_client_ptr->getURDFString();
    std::cout << "Created file " << filename << ".\n";

    file.close();

    return 0;
}
