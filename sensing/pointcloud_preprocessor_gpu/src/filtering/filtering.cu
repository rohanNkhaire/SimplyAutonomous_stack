#include "pointcloud_preprocessor_gpu/filtering/kernel.hpp"

__global__ void filter_pointcloud(int* filtered_points, int* points, size_t points_size,
        float min_x_range, float max_x_range,
        float min_y_range, float max_y_range,
        float min_z_range, float max_z_range)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if(point_idx >= points_size) return;

  int4 point = ((int4*)points)[point_idx];

  if(point.x<min_x_range||point.x>=max_x_range
    || point.y<min_y_range||point.y>=max_y_range
    || point.z<min_z_range||point.z>=max_z_range) return;

	filtered_points[point_idx+0] = point.x;
	filtered_points[point_idx+1] = point.y;
	filtered_points[point_idx+2] = point.z;
	filtered_points[point_idx+3] = point.w;
	
}

cudaError_t generate_filtered_pointcloud(int* filtered_points, int* points, size_t points_size,
        float min_x_range, float max_x_range,
        float min_y_range, float max_y_range,
        float min_z_range, float max_z_range,
        cudaStream_t stream = 0)
{
	int threadNum = 256;
  dim3 blocks((points_size+threadNum-1)/threadNum);
  dim3 threads(threadNum);
  filter_pointcloud<<<blocks, threads, 0, stream>>>
    (filtered_points, points, points_size,
        min_x_range, max_x_range,
        min_y_range, max_y_range,
        min_z_range, max_z_range);
  cudaError_t err = cudaGetLastError();

  return err;
}