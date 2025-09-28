// https://www.theconstruct.ai/how-to-integrate-opencv-with-a-ros2-c-node/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class VideoPublisher : public rclcpp::Node
{
  public:
    VideoPublisher() : Node("video_publisher_node"), count_(0)
    {
		publisher_txt = this->create_publisher<std_msgs::msg::String>("topic", 10);
		publisher_img = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
      
		my_timer      = this->create_wall_timer(
								std::chrono::milliseconds(33), 
								std::bind(&VideoPublisher::timer_callback, this)
								);
	
		int capture_width  = 640;		// capture_width = 1280 ;
		int capture_height = 480;		// capture_height = 720 ;
		int display_width  = 640;		// display_width = 1280 ;
		int display_height = 480;		// display_height = 720 ;
		int framerate = 20;			    // framerate = 30 ;
		int flip_method = 2;
		
		pipeline = gstreamer_pipeline(
						capture_width,
						capture_height,
						display_width,
						display_height,
						framerate,
						flip_method
						);
									
		// Open camera 
        video_stream.open(pipeline, cv::CAP_GSTREAMER);

        if (!video_stream.isOpened()) 
			{
            RCLCPP_ERROR (this->get_logger(), "Failed to open camera");
			}
		else
			RCLCPP_INFO (this->get_logger(), "Camera was succesfully open");
    }

  private:
    void timer_callback()
		{
		if (!video_stream.read(img)) 
			{
			RCLCPP_ERROR (this->get_logger(), "Frame capture read error");
			return;
			}
			
		// Convert cv::Mat to ROS2 Image
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";

        auto msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        publisher_img->publish(*msg);
			
		//auto message = std_msgs::msg::String();
		//message.data = "Hello, world! " + std::to_string(count_++);
		//RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
		//publisher_txt->publish(message);
		}
    
    std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
		   }
    
    rclcpp::TimerBase::SharedPtr my_timer;
    rclcpp::Publisher  <std_msgs::msg::String>  ::SharedPtr publisher_txt;
    rclcpp::Publisher <sensor_msgs::msg::Image> ::SharedPtr publisher_img;

    size_t count_;
    
    std::string pipeline;
    cv::VideoCapture video_stream;
    cv::Mat img;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VideoPublisher>());
  rclcpp::shutdown();
  
  return 0;
}

