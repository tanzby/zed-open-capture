#include <iostream>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>

#include "videocapture.hpp"
#include "sensorcapture.hpp"

class ZEDWrapper
{
public:

    ZEDWrapper(ros::NodeHandle nh);
    ~ZEDWrapper();

private:

    // most recent sync imu timestamp
    uint64_t mcu_sync_ts_ = 0;
    
    // ts, accel, gyro
    std::string imu_info_[3];

    // protect multi-read of imu infor
    std::mutex imu_mutex_;

    std::shared_ptr<std::thread> sensor_cap_thread_;
    std::shared_ptr<std::thread> video_cap_thread_;

    std::shared_ptr<sl_oc::sensors::SensorCapture> sensor_cap_;
    std::shared_ptr<sl_oc::video::VideoCapture>    video_cap_;

    ros::Publisher pub_imu_;
    image_transport::Publisher pub_left_;
    image_transport::Publisher pub_right_;

    void init();

    void sensor_callback();

    void video_callback();
    
};

void ZEDWrapper::init() {

    // Set the verbose level
    sl_oc::VERBOSITY verbose = sl_oc::VERBOSITY::ERROR;

    // ----> Set the video parameters
    sl_oc::video::VideoParams params;
    params.res = sl_oc::video::RESOLUTION::HD720;
    params.fps = sl_oc::video::FPS::FPS_30;
    params.verbose = verbose;
    // <---- Video parameters

    // ----> Create a Video Capture object
    video_cap_ = std::make_shared<sl_oc::video::VideoCapture>(params);
    if( !video_cap_->initializeVideo(-1) )
    {
        std::stringstream ss;
        ss << "Cannot open camera video capture" << std::endl
            << "Try to enable verbose to get more info";
        throw std::runtime_error(ss.str());
    }

    auto cam_sn = video_cap_->getSerialNumber();

    ROS_INFO_STREAM("Video Capture connected to camera sn: " << cam_sn);
    // <---- Create a Video Capture object

    // ----> Create a Sensors Capture object
    sensor_cap_ = std::make_shared<sl_oc::sensors::SensorCapture>(verbose);
    if( !sensor_cap_->initializeSensors(cam_sn) ) // Note: we use the serial number acquired by the VideoCapture object
    {
        std::stringstream ss;
        ss << "Cannot open sensor capture" << std::endl
           << "Try to enable verbose to get more info";
        throw std::runtime_error(ss.str());
    }

    ROS_INFO_STREAM("Sensors Capture connected to camera sn: " << sensor_cap_->getSerialNumber());

    // Start the sensor capture thread. Note: since sensor data can be retrieved at 400Hz and video data frequency is
    // minor (max 100Hz), we use a separated thread for sensors.
    sensor_cap_thread_ = std::make_shared<std::thread>(&ZEDWrapper::sensor_callback, this);
    // <---- Create Sensors Capture

    // ----> Enable video/sensors synchronization
    video_cap_->enableSensorSync(sensor_cap_.get());
    // <---- Enable video/sensors synchronization

    video_cap_thread_ = std::make_shared<std::thread>(&ZEDWrapper::video_callback, this);
}

void ZEDWrapper::sensor_callback() {
    uint64_t last_imu_ts = 0;
    uint64_t msg_count = 0;
    
    while(ros::ok())
    {
        // ----> Get IMU data
        const sl_oc::sensors::data::Imu imu_data = sensor_cap_->getLastIMUData(2000);

        // Process data only if valid
        if(imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL ) // Uncomment to use only data syncronized with the video frames
        {
            // ----> Create and Send ROS msg
            sensor_msgs::ImuPtr imu_raw_msg = boost::make_shared<sensor_msgs::Imu>();
            imu_raw_msg->header.frame_id = "/base_link";
            imu_raw_msg->header.seq = msg_count++;
            imu_raw_msg->header.stamp.fromNSec(imu_data.timestamp);
            imu_raw_msg->angular_velocity.x = imu_data.aX;
            imu_raw_msg->angular_velocity.y = imu_data.aY;
            imu_raw_msg->angular_velocity.z = imu_data.aZ;
            imu_raw_msg->linear_acceleration.x = imu_data.gX;
            imu_raw_msg->linear_acceleration.y = imu_data.gY;
            imu_raw_msg->linear_acceleration.z = imu_data.gZ;
            imu_raw_msg->linear_acceleration_covariance[0] = -1;
            imu_raw_msg->angular_velocity_covariance[0] = -1;
            imu_raw_msg->orientation_covariance[0] = -1;
            pub_imu_.publish(imu_raw_msg);
            // <---- ROS msg
            
            
            // ----> Data info to be displayed
            std::stringstream timestamp;
            std::stringstream accel;
            std::stringstream gyro;

            timestamp << std::fixed << std::setprecision(9) << "IMU timestamp:   " << static_cast<double>(imu_data.timestamp)/1e9<< " sec" ;
            if(last_imu_ts!=0)
                timestamp << std::fixed << std::setprecision(1)  << " [" << 1e9/static_cast<float>(imu_data.timestamp-last_imu_ts) << " Hz]";
            last_imu_ts = imu_data.timestamp;

            accel << std::fixed << std::showpos << std::setprecision(4) << " * Accel: " << imu_data.aX << " " << imu_data.aY << " " << imu_data.aZ << " [m/s^2]";
            gyro << std::fixed << std::showpos << std::setprecision(4) << " * Gyro: " << imu_data.gX << " " << imu_data.gY << " " << imu_data.gZ << " [deg/s]";
            // <---- Data info to be displayed

            // Mutex to not overwrite data while diplaying them
            imu_mutex_.lock();

            imu_info_[0] = timestamp.str();
            imu_info_[1] = accel.str();
            imu_info_[2] = gyro.str();

            // ----> Timestamp of the synchronized data
            if(imu_data.sync)
            {
                mcu_sync_ts_ = imu_data.timestamp;
            }
            // <---- Timestamp of the synchronized data

            imu_mutex_.unlock();
        }
        // <---- Get IMU data
    }
}

void ZEDWrapper::video_callback() {

    // ----> Init OpenCV RGB frame
    int w, h;
    video_cap_->getFrameSize(w,h);

    cv::Size display_resolution(1024, 576);

    switch(h)
    {
        default:
        case 376: // sl_oc::video::RESOLUTION::VGA
            display_resolution.width = w;
            display_resolution.height = h;
            break;
        case 720: // sl_oc::video::RESOLUTION::HD720
            display_resolution.width = w*0.6;
            display_resolution.height = h*0.6;
            break;
        case 1080: // sl_oc::video::RESOLUTION::HD1080
        case 1242: // sl_oc::video::RESOLUTION::HD2K
            display_resolution.width = w*0.4;
            display_resolution.height = h*0.4;
            break;
    }

    int h_data = 70;
    cv::Mat frameDisplay(display_resolution.height + h_data, display_resolution.width,CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat frameData = frameDisplay(cv::Rect(0,0, display_resolution.width, h_data));
    cv::Mat frameBGRDisplay = frameDisplay(cv::Rect(0,h_data, display_resolution.width, display_resolution.height));
    cv::Mat frameBGR(h, w, CV_8UC3, cv::Scalar(0,0,0));

    cv::Rect left_part (0, 0, w/2, h);
    cv::Rect right_part(w/2, 0, w/2, h);

    // <---- Init OpenCV RGB frame
    
    uint64_t last_timestamp = 0;
    uint64_t msg_count = 0;

    float frame_fps=0;

    // Infinite grabbing loop
    while (ros::ok())
    {
        // ----> Get Video frame
        // Get last available frame
        const sl_oc::video::Frame frame =video_cap_->getLastFrame(1);

        // If the frame is valid we can update it
        std::stringstream videoTs;
        if(frame.data!=nullptr && frame.timestamp!=last_timestamp)
        {
            frame_fps = 1e9/static_cast<float>(frame.timestamp-last_timestamp);
            last_timestamp = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV( frame.height, frame.width, CV_8UC2, frame.data);
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            std_msgs::Header header;
            header.seq = msg_count++;
            header.stamp.fromNSec(frame.timestamp);

            header.frame_id = "left_cam";
            auto left_msg  = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frameBGR(left_part)).toImageMsg();

            header.frame_id = "right_cam";
            auto right_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frameBGR(right_part)).toImageMsg();

            pub_left_.publish(left_msg);
            pub_right_.publish(right_msg);
        }
        // <---- Get Video frame

        // ----> Video Debug information
        videoTs << std::fixed << std::setprecision(9) << "Video timestamp: " << static_cast<double>(last_timestamp)/1e9<< " sec" ;
        if( last_timestamp!=0 )
            videoTs << std::fixed << std::setprecision(1)  << " [" << frame_fps << " Hz]";
        // <---- Video Debug information

        // ----> Display frame with info
        if(frame.data!=nullptr)
        {
            frameData.setTo(0);

            // Video info
            cv::putText( frameData, videoTs.str(), cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(241,240,236));

            // IMU info
            imu_mutex_.lock();
            cv::putText( frameData, imu_info_[0], cv::Point(10, 35),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(241,240,236));

            // Timestamp offset info
            std::stringstream offsetStr;
            double offset = (static_cast<double>(frame.timestamp)-static_cast<double>(mcu_sync_ts_))/1e9;
            offsetStr << std::fixed << std::setprecision(9) << std::showpos << "Timestamp offset: " << offset << " sec [video-sensors]";
            cv::putText( frameData, offsetStr.str().c_str(), cv::Point(10, 50),cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(241,240,236));

            // Average timestamp offset info (we wait at least 200 frames to be sure that offset is stable)
            if( frame.frame_id>200 )
            {
                static double sum=0;
                static int count=0;

                sum += offset;
                double avg_offset=sum/(++count);

                std::stringstream avgOffsetStr;
                avgOffsetStr << std::fixed << std::setprecision(9) << std::showpos << "Avg timestamp offset: " << avg_offset << " sec";
                cv::putText( frameData, avgOffsetStr.str().c_str(), cv::Point(10,62),cv::FONT_HERSHEY_SIMPLEX,0.35, cv::Scalar(241, 240,236));
            }

            // IMU values
            cv::putText( frameData, "Inertial sensor data:", cv::Point(display_resolution.width/2,20),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            cv::putText( frameData, imu_info_[1], cv::Point(display_resolution.width/2+15,42),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            cv::putText( frameData, imu_info_[2], cv::Point(display_resolution.width/2+15, 62),cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(241, 240,236));
            imu_mutex_.unlock();

            // Resize Image for display
            cv::resize(frameBGR, frameBGRDisplay, display_resolution);
            // Display image
            cv::imshow( "Stream RGB", frameDisplay );
        }
        // <---- Display frame with info

        // ----> Keyboard handling
        int key = cv::waitKey(1);

        if( key != -1 )
        {
            // Quit
            if(key=='q' || key=='Q'|| key==27)
            {
                ros::shutdown();
                break;
            }
        }
        // <---- Keyboard handling
    }
    
}

ZEDWrapper::ZEDWrapper(ros::NodeHandle nh) {
    // read ros param

    pub_imu_ = nh.advertise<sensor_msgs::Imu>("imu/data", 100, false);

    image_transport::ImageTransport it(nh);
    pub_left_  = it.advertise("left/image_raw", 1);
    pub_right_ = it.advertise("right/image_raw", 1);

    init();
}

ZEDWrapper::~ZEDWrapper() {
    sensor_cap_thread_->join();
    video_cap_thread_->join();
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "zed_node");
    ros::NodeHandle nh("~");

    ZEDWrapper app(nh);

    ros::spin();
    return 0;
}