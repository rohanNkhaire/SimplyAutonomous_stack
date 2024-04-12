#ifndef CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_
#define CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_

#include "pointcloud_preprocessor_gpu/filtering/kernel.hpp"
#include <rclcpp/rclcpp.hpp>


// Messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

// TF
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// PCL
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>

#include <memory>



class CropBoxFilter : public rclcpp::Node
{
public:
  CropBoxFilter();

private:
	
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_pcd_sub_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcd_pub_;

	// Callbacks
	void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr);

	// Parameters
	float max_x, min_x;
	float max_y, min_y;
	float max_z, min_z;
	std::string tf_output_frame_;

	// Variables
	sensor_msgs::msg::PointCloud2 pcd_data_;
	sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
	
	// TF
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	// Functions
	void setupTF();
	void generateROSPCD(float*, size_t, const sensor_msgs::msg::PointCloud2::SharedPtr&);


};

#endif //CROP_BOX_FILTER__CROP_BOX_FILTER_HPP_