#ifndef PREPROCESS_KERNEL_H_
#define PREPROCESS_KERNEL_H_

#include <cuda.hpp>
#include <iostream>
#include <cuda_fp16.hpp>
#include <cuda_runtime_api.h>

typedef enum
{
    STATUS_SUCCESS = 0,
    STATUS_FAILURE = 1,
    STATUS_BAD_PARAM = 2,
    STATUS_NOT_SUPPORTED = 3,
    STATUS_NOT_INITIALIZED = 4
} pluginStatus_t;

#define checkCudaErrors(status)                                   \
{                                                                 \
  if (status != 0)                                                \
  {                                                               \
    std::cout << "Cuda failure: " << cudaGetErrorString(status)   \
              << " at line " << __LINE__                          \
              << " in file " << __FILE__                          \
              << " error status: " << status                      \
              << std::endl;                                       \
              abort();                                            \
    }                                                             \
}

cudaError_t generate_filtered_pointcloud(int* filtered_points, int* points, size_t points_size,
        float min_x_range, float max_x_range,
        float min_y_range, float max_y_range,
        float min_z_range, float max_z_range,
        cudaStream_t stream = 0);
       


#endif // PREPROCESS_KERNEL_H