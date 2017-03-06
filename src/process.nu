#include <opencv2/core/core.hpp>
#include <opencv/highgui/highgui.hpp>
#include <opencv/opencv.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>


__global__ void gen_cloud(const uchar4* d_red_img, pcl::cuda::PointCloudAOS

cv::Mat red_img;

uchar4	*d_red_img;

void process(const cv::Mat& red_src, const int numPix){
    
    checkCudaErrors(cudaMalloc(d_red_img, sizeof(uchar4)*numPix));
    checkCudaErrors(cudaMemcpy(*d_red_img, *red_img, sizeof(uchar4)*numPix, cudaMemcpyHostToDevice));
    
	pcl::cuda::PointCloudAOS<pcl::PointCloud::PointXYZRGB> red_cloud_gpu;
	
        

    pcl::PointCloud<pcl::PointCloud::PointXYZRGB> red_cloud_cpu;
	
	
