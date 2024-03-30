#include "pointcloud_preprocessor_gpu/filtering/filtering.hpp"


CropBoxFilter::CropBoxFilter() : Node("crop_box_filter")
{
	input_pcd_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("", rclcpp::QoS{5},
																											std::bind(&CropBoxFilter::pointcloud_callback, this, std::placeholders::_1));

	filtered_pcd_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/cropbox_pointcloud", rclcpp::Qos{10});

	setupTF();																										
}


void CropBoxFilter::setupTF()
{

}

void CropBoxFilter::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	cudaEvent_t start, stop;
  float elapsedTime = 0.0f;
  cudaStream_t stream = NULL;

  checkCudaErrors(cudaEventCreate(&start));
  checkCudaErrors(cudaEventCreate(&stop));
  checkCudaErrors(cudaStreamCreate(&stream));

  int* points = (int*)msg->data.data();
  size_t height = msg->height;
  size_t width = msg->width;
  size_t row_step = msg->row_step;
  size_t length = row_step * height;
  size_t points_size = length/sizeof(int)/4;

  int* points_data = nullptr;
	int* filtered_points_data = nullptr;
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

	// Make filtered PCD
	sensor_msgs::msg::PointCloud2 filtered_pcd;
	filtered_pcd.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
	filtered_pcd.header.frame_id = "ego_vehicle/lidar";
	filtered_pcd.height = height;
	filtered_pcd.width = width;
	filtered_pcd.row_step = row_step;
	filtered_pcd.data = filtered_points_data;

	// Transform PCD

	filtered_pcd_pub_->publish()														

  cudaEventRecord(stop, stream);
  cudaEventSynchronize(stop);
  cudaEventElapsedTime(&elapsedTime, start, stop);
  std::cout<<"TIME: crop box filtering: "<< elapsedTime <<" ms." <<std::endl;

  checkCudaErrors(cudaFree(points_data));

  checkCudaErrors(cudaEventDestroy(start));
  checkCudaErrors(cudaEventDestroy(stop));
  checkCudaErrors(cudaStreamDestroy(stream));
}

