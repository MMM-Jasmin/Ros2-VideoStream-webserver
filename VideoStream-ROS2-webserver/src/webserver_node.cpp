#include "webserver_node.hpp"

/**
 * @brief Contructor.
 */
WebserverNode::WebserverNode(const std::string &name) : Node(name, rclcpp::NodeOptions().use_intra_process_comms(true)) {

	this->declare_parameter("rotation", 0);
	this->declare_parameter("debug", false);
	this->declare_parameter("topic", "");
	this->declare_parameter("port", 7777);
	
}

/**
 * @brief Initialize image node.
 */
void WebserverNode::init() {

	std::string ros_topic;
	this->get_parameter("topic", ros_topic);

	int output_port;
	this->get_parameter("port", output_port);

	// Create mjpeg writer with a choosen port.
	// TODO maybe parameterize the port ???
	m_mjpeg_writer_ptr = new MJPEGWriter(output_port);
	//center_writer_thr = new std::thread(write_to_mjpeg_writer,std::ref(post_draw_image));
	//mjpeg_writer_ptr->start(); //Starts the HTTP Server on the selected port

	m_image_small_subscription = this->create_subscription<sensor_msgs::msg::Image>( ros_topic, m_qos_profile, std::bind(&WebserverNode::imageSmallCallback, this, std::placeholders::_1));
	//cv::namedWindow(m_window_name_image_small, cv::WINDOW_AUTOSIZE);


}


/**
 * @brief Callback function for reveived image message.
 * @param img_msg Received image message
 */
void WebserverNode::imageSmallCallback(sensor_msgs::msg::Image::SharedPtr img_msg) {

	int rotation;
	this->get_parameter("rotation", rotation);

	cv::Size image_size(static_cast<int>(img_msg->width), static_cast<int>(img_msg->height));
	cv::Mat color_image(image_size, CV_8UC3, (void *)img_msg->data.data(), cv::Mat::AUTO_STEP);
	
	if (rotation == 90)
		cv::rotate(color_image, color_image, cv::ROTATE_90_CLOCKWISE);
	else if (rotation == 180)
		cv::rotate(color_image, color_image, cv::ROTATE_180);
	else if (rotation == 270)
		cv::rotate(color_image, color_image, cv::ROTATE_90_COUNTERCLOCKWISE); 

	//cv::setWindowTitle(m_window_name_image_small, std::to_string(m_loop_duration_image_small));
	//cv::setWindowTitle(m_window_name, std::to_string(0.0));
	cv::cvtColor(color_image, color_image, cv::COLOR_RGB2BGR);

	m_mjpeg_writer_ptr->write(color_image);
	if(!m_webservice_started){
		m_mjpeg_writer_ptr->start(); //Starts the HTTP Server on the selected port
		m_webservice_started = true;
	}

	//imshow(m_window_name_image_small, color_image);

	//if (!(cv::waitKey(1) < 0 && cv::getWindowProperty(m_window_name_image_small, cv::WND_PROP_AUTOSIZE) >= 0))
	//	rclcpp::shutdown();

	//m_loop_duration_image_small = (hires_clock::now() - m_callback_time_image_small).count() / 1e6;
	//m_callback_time_image_small = hires_clock::now();
}

/**
 * @brief Declare DepthFusion ros node parameters.
 */
void WebserverNode::declareNodeParameters()
{



}