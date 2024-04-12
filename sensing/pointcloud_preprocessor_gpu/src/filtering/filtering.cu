#include "pointcloud_preprocessor_gpu/filtering/kernel.hpp"

__global__ void filter_pointcloud(float* filtered_points, float* points, size_t points_size,
        float min_x_range, float max_x_range,
        float min_y_range, float max_y_range,
        float min_z_range, float max_z_range)
{
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if(point_idx >= points_size) return;

  float4 point = ((float4*)points)[point_idx];

  if(point.x>min_x_range && point.x<max_x_range
    && point.y>min_y_range && point.y<max_y_range
    && point.z>min_z_range && point.z<max_z_range) return;


  float* address = filtered_points + (point_idx)*4;

  atomicExch(address+0, point.x);
  atomicExch(address+1, point.y);
  atomicExch(address+2, point.z);
  atomicExch(address+3, point.w);
 
}

cudaError_t generate_filtered_pointcloud(float* filtered_points, float* points, size_t points_size,
        float min_x_range, float max_x_range,
        float min_y_range, float max_y_range,
        float min_z_range, float max_z_range,
        cudaStream_t stream)
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