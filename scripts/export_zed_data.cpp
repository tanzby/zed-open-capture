#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <rosbag/view.h>
#include <rosbag/bag.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    po::options_description desc("Allowed options");

    desc.add_options()
        ("help,h", "produce help message")
        ("bag,b", po::value<std::string>()->default_value(""),"bag file you want to extract")
        ("save,s", po::value<std::string>()->default_value("./"),"path data will extract to")
        ("imu_topic,i", po::value<std::string>()->default_value("/zed_node/imu/data"))
        ("left_topic,l", po::value<std::string>()->default_value("/zed_node/left/image_raw/compressed"))
        ("right_topic,r", po::value<std::string>()->default_value("/zed_node/right/image_raw/compressed"))
        ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }

    struct Options
    {
        std::string filename;
        std::string directory;
        std::string imu_topic;
        std::string left_topic;
        std::string right_topic;
    };

    Options opts;

    if (vm.count("bag"))
        opts.filename = vm["bag"].as<std::string>();
    if (vm.count("save"))
        opts.directory = vm["save"].as<std::string>();
    if (vm.count("imu_topic"))
        opts.imu_topic = vm["imu_topic"].as<std::string>();
    if (vm.count("left_topic"))
        opts.left_topic = vm["left_topic"].as<std::string>();
    if (vm.count("right_topic"))
        opts.right_topic = vm["right_topic"].as<std::string>();

    if (opts.filename.empty()) {
        throw std::logic_error("you must specify your bag file, cannot be empty");
    }

    //! check dir
    if (opts.directory.back() != '/')
        opts.directory = opts.directory + '/';

    auto results_dir = opts.directory + opts.filename.substr(0, opts.filename.length()-4) + "_results";

    if (fs::exists(results_dir))
        fs::remove_all(results_dir);

    fs::create_directories(results_dir);

    auto left_image_dir = results_dir + "/left/";
    auto right_image_dir = results_dir + "/right/";

    fs::create_directories(left_image_dir);
    fs::create_directories(right_image_dir);

    //! Read bag file
    std::cout << "Reading the rosbag file from " << opts.filename << std::endl;

    rosbag::Bag bag;
    bag.open(opts.filename, rosbag::bagmode::Read);

    rosbag::View view(bag);

    std::cout << "Writing sensor_msgs::Imu data to CSV, Extract Image to png" << std::endl;

    std::ofstream of_imu_data(results_dir + "/" + opts.filename + "_imu_data.csv", std::ios::out);
    std::ofstream of_left_ts_data(results_dir + "/" + opts.filename + "_left_timestamp.csv", std::ios::out);
    std::ofstream of_right_ts_data(results_dir + "/" + opts.filename + "_right_timestamp.csv", std::ios::out);

    of_imu_data << "# timestamp ax ay az gx gy gz" << std::endl;
    of_left_ts_data << "# timestamp" << std::endl;
    of_right_ts_data << "# timestamp" << std::endl;

    for (auto& m: view)
    {
        const auto& topic = m.getTopic();
        if (topic == opts.imu_topic)
        {
            sensor_msgs::ImuConstPtr msg = m.instantiate<sensor_msgs::Imu>();

            of_imu_data  << msg->header.stamp.toNSec() << ","
                << msg->angular_velocity.x << ","
                << msg->angular_velocity.y << ","
                << msg->angular_velocity.z << ","
                << msg->linear_acceleration.x << ","
                << msg->linear_acceleration.y << ","
                << msg->linear_acceleration.z << std::endl;
        }
        else if (topic == opts.left_topic)
        {
            sensor_msgs::CompressedImage::ConstPtr msg = m.instantiate<sensor_msgs::CompressedImage>();

            of_left_ts_data << msg->header.stamp.toNSec() << std::endl;

            std::stringstream ss;
            ss << left_image_dir << msg->header.stamp.toNSec() << ".png";

            auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            cv::imwrite(ss.str(), img);
        }
        else if (topic == opts.right_topic)
        {
            sensor_msgs::CompressedImage::ConstPtr msg = m.instantiate<sensor_msgs::CompressedImage>();

            of_right_ts_data << msg->header.stamp.toNSec() << std::endl;

            std::stringstream ss;
            ss << right_image_dir << msg->header.stamp.toNSec() << ".png";

            auto img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
            cv::imwrite(ss.str(), img);
        }
    }

    bag.close();
    of_right_ts_data.close();
    of_left_ts_data.close();
    of_right_ts_data.close();

    std::cout << "Finished extracting data!" << std::endl;

    return 0;
}