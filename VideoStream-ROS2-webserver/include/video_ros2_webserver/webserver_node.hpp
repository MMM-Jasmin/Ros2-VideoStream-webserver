#pragma once
// SYSTEM
#include <chrono>
#include <iostream>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// OPENCV
#include <opencv2/opencv.hpp>

// PROJECT
#include "camera_interfaces/msg/depth_frameset.hpp"
#include "MJPEGWriter.hpp"

/**
 * @brief Image viewer node class for receiving and visualizing fused image.
 */
class WebserverNode : public rclcpp::Node
{
	typedef std::chrono::high_resolution_clock::time_point time_point;
	typedef std::chrono::high_resolution_clock hires_clock;

public:
	WebserverNode(const std::string &name);
	void init();

private:

	void declareNodeParameters();

	MJPEGWriter* m_mjpeg_writer_ptr;
	bool m_webservice_started = false;

	std::string m_window_name_image_small	= "Image_small_Frame";

	time_point m_callback_time = hires_clock::now();
	time_point m_callback_time_image_small = hires_clock::now();
	time_point m_callback_time_depth = hires_clock::now();

	double m_loop_duration = 0.0;
	double m_loop_duration_image_small = 0.0;
	double m_loop_duration_depth = 0.0;

	//rclcpp::QoS m_qos_profile = rclcpp::SensorDataQoS();
	rclcpp::QoS m_qos_profile = rclcpp::SystemDefaultsQoS();

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_small_subscription;

	void imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
	
};
