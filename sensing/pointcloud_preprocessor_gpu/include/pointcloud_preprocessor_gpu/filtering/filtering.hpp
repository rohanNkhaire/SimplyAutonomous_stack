#ifndef CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_
#define CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_

#include "kernel.hpp"
#include <rclcpp/rclcpp.hpp>

// Messages
#include <sensor_msgs/msg/point_cloud2.hpp>

// TF


#include <memory>

class CropBoxFilter : public rclcpp::Node
{
public:
  CropBoxFilter();
  ~CropBoxFilter();

private:
	
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_pcd_sub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcd_pub_;

	// Callbacks
	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr);

	// Parameters
	double max_x, min_x;
	double max_y, min_y;
	double max_z, min_z;

	// Variables
	sensor_msgs::msg::PointCloud2 pcd_data_;

	// TF

	// Functions
	void setupTF();


};

#endif //CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_