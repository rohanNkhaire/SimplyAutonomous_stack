#include "pointcloud_preprocessor_gpu/filtering/filtering.hpp"

CropBoxFilter::CropBoxFilter() : Node("crop_box_filter")
{
	input_pcd_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("/carla/ego_vehicle/lidar", rclcpp::QoS{5},
																											std::bind(&CropBoxFilter::pointcloud_callback, this, std::placeholders::_1));

	filtered_pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cropbox_pointcloud", rclcpp::QoS{10});

	setupTF();

  // Declaring parameters
  {
    min_x = static_cast<float>(declare_parameter("min_x", -1.0));
    min_y = static_cast<float>(declare_parameter("min_y", -1.0));
    min_z = static_cast<float>(declare_parameter("min_z", -1.0));
    max_x = static_cast<float>(declare_parameter("max_x", 1.0));
    max_y = static_cast<float>(declare_parameter("max_y", 1.0));
    max_z = static_cast<float>(declare_parameter("max_z", 1.0));
    tf_output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
  }
}

void CropBoxFilter::setupTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void CropBoxFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	cudaEvent_t start, stop;
  float elapsedTime = 0.0f;
  cudaStream_t stream = NULL;

  checkCudaErrors(cudaEventCreate(&start));
  checkCudaErrors(cudaEventCreate(&stop));
  checkCudaErrors(cudaStreamCreate(&stream));

  float* points = (float*)msg->data.data();
  size_t height = msg->height;
  size_t width = msg->width;
  size_t row_step = msg->row_step;
  size_t length = row_step * height;
  size_t points_size = length/sizeof(int)/4;

  float* points_data = nullptr;
	float* filtered_points_data = nullptr;
  unsigned int points_data_size = points_size * 4 * sizeof(int);
  
  checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
	checkCudaErrors(cudaMallocManaged((void **)&filtered_points_data, points_data_size));
  checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
  checkCudaErrors(cudaDeviceSynchronize());

  cudaEventRecord(start, stream);

  // Do filtering
	generate_filtered_pointcloud(filtered_points_data, points_data, points_size, 
															min_x, max_x, min_y, max_y,
															min_z, max_z, stream);                                                                                										
  
  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  //RCLCPP_WARN(get_logger(), "TIME: crop box filtering:%f ms", elapsedTime);  

  // Make filtered PCD
  pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

  // Fill in PCD data
  generateROSPCD(filtered_points_data, points_size, pc2_msg_);

  pc2_msg_->header = msg->header;

	// Transform PCD
  /*
  sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (pcl_ros::transformPointCloud("ego_vehicle", *pc2_msg_, transformed_cloud, *tf_buffer_)) {
        transformed_cloud.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        transformed_cloud.header.frame_id = tf_output_frame_;
    }
  */
 
  // Publish PCD
	filtered_pcd_pub_->publish(*pc2_msg_);	

  checkCudaErrors(cudaFree(points_data));
  checkCudaErrors(cudaFree(filtered_points_data));

  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));
}

void CropBoxFilter::generateROSPCD(float* filtered_cloud_ptr, size_t points_size, const sensor_msgs::msg::PointCloud2::SharedPtr& pcd2ros_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>());
  for (size_t i = 0; i < points_size; ++i)
  {
    if (filtered_cloud_ptr[(i*4)+3] != 0)
    {
      pcl::PointXYZI pt;
      pt.x = filtered_cloud_ptr[i*4];
      pt.y = filtered_cloud_ptr[(i*4)+1];
      pt.z = filtered_cloud_ptr[(i*4)+2];
      pt.intensity = filtered_cloud_ptr[(i*4)+3]; 

      cloud->points.push_back(pt);
    }
  }

  pcl::toROSMsg(*cloud, *pcd2ros_msg);
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CropBoxFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}