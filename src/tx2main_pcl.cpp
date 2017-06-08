///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/************************************************************************************
 ** This sample demonstrates how to use PCL (Point Cloud Library) with the ZED SDK **
 ************************************************************************************/

// ZED includes
#include <sl/Camera.hpp>

using namespace std;
// PCL includes
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

// Sample includes
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "color_filter.h"

// Namespace
using namespace sl;

// Global instance (ZED, Mat, callback)
Camera zed;
Mat data_cloud;
std::thread zed_callback;
std::mutex mutex_input;
bool stop_signal;
bool has_data;

// Sample functions
void startZED();
void run();
void closeZED();
shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
inline float convertColor(float colorIn);
cv::Mat slMat2cvMat(sl::Mat& input);
// Main process

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_ros_node");
    ros::NodeHandle nh;
    std::string pcl_cloud_topic = "point_cloud/white_cloud";
    std::string pcl_frame_id = "/zed_current_frame";
    ros::Publisher pcl_publisher = nh.advertise<sensor_msgs::PointCloud2> (pcl_cloud_topic, 2);
    //if (argc > 2) {
    //    cout << "Only the path of a SVO can be passed in arg" << endl;
    //    return -1;
    //}

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_VGA;// RESOLUTION_HD720;
    if (argc == 2) init_params.svo_input_filename = argv[1];
    init_params.coordinate_units = UNIT_METER;
    init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.camera_fps = 30;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << errorCode2str(err) << endl;
        return 1;
    }

    // Allocate PCL point cloud at the resolution
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    p_pcl_point_cloud->points.resize(zed.getResolution().area());

    // Create the PCL point cloud visualizer
    //shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);

    // Start ZED callback
    startZED();
    ColorFilter color_filter;


    ros::Rate loop_rate(30);
    while (nh.ok()) {
    // Loop until viewer catches the stop signal
    //while (!viewer->wasStopped()) {
        
        cv::Mat cloud_image;
        sl::Mat  host_image;
    ROS_INFO("herenow1a");
                mutex_input.lock(); // To prevent from data corruption
        zed.retrieveImage(host_image,sl::VIEW_LEFT);
                    mutex_input.unlock();
        cloud_image = slMat2cvMat(host_image);
    ROS_INFO("herenow2a");
	    cv::gpu::GpuMat cv_filteredImage = color_filter.findLines(cloud_image);
    ROS_INFO("herenow3a");
	    cv_filteredImage.download(cloud_image); //move image to cpu memory
    ROS_INFO("herenow4a");
        // Try to lock the data if possible (not in use). Otherwise, do nothing.
        if (mutex_input.try_lock()) {
            float *p_data_cloud = data_cloud.getPtr<float>();
            int index = 0;
            // Check and adjust points for PCL format
            for (auto &it : p_pcl_point_cloud->points) {       
                int wl_point = cloud_image.at<int>(index/4/cloud_image.rows,index/4%cloud_image.cols);
	            if (wl_point > 10){
                    float X = p_data_cloud[index];
                    if (!isValidMeasure(X)) // Checking if it's a valid point
                        it.x = it.y = it.z = it.rgb = 0;
                    else {
                        it.x = X;
                        it.y = p_data_cloud[index + 1];
                        it.z = p_data_cloud[index + 2];
                        it.rgb = convertColor(p_data_cloud[index + 3]); // Convert a 32bits float into a pcl .rgb format
                    }
                    index += 4;
                }
            }
            // Unlock data and update Point cloud
            mutex_input.unlock();
            //viewer->updatePointCloud(p_pcl_point_cloud);
            //viewer->spinOnce(15);
        } else
            sleep_ms(1);
            		    
        sensor_msgs::PointCloud2 pcl_msg;
	    pcl::toROSMsg(*p_pcl_point_cloud, pcl_msg); // Convert the point cloud to a ROS message
   		pcl_msg.header.frame_id = pcl_frame_id; // Set the header values of the ROS message
   		pcl_msg.header.stamp = ros::Time::now();
   		pcl_msg.is_bigendian = false;
   		pcl_msg.is_dense = false;
   		pcl_publisher.publish(pcl_msg);
            
    ros::spinOnce();
    loop_rate.sleep();
    }

    // Close the viewer
    //viewer->close();

    // Close the zed
    closeZED();

    return 0;
}

/**
 *  This functions start the ZED's thread that grab images and data.
 **/
void startZED() {
    // Start the thread for grabbing ZED data
    stop_signal = false;
    has_data = false;
    zed_callback = std::thread(run);

    //Wait for data to be grabbed
    while (!has_data)
        sleep_ms(1);
}

/**
 *  This function loops to get the point cloud from the ZED. It can be considered as a callback.
 **/
void run() {
    while (!stop_signal) {
        if (zed.grab(SENSING_MODE_STANDARD) == SUCCESS) {
            mutex_input.lock(); // To prevent from data corruption
            zed.retrieveMeasure(data_cloud, MEASURE_XYZRGBA);
            mutex_input.unlock();
            has_data = true;
        }
        sleep_ms(1);
    }
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void closeZED() {
    // Stop the thread
    stop_signal = true;
    zed_callback.join();
    zed.close();
}

/**
 *  This function creates a PCL visualizer
 **/
//shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    //shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer"));
    //viewer->setBackgroundColor(0.12, 0.12, 0.12);
    //pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    //viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    //viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
    //return ();
//}

/**
 *  This function convert a RGBA color packed into a packed RGBA PCL compatible format
 **/
inline float convertColor(float colorIn) {
    uint32_t color_uint = *(uint32_t*) & colorIn;
    unsigned char* color_uchar = (unsigned char*) &color_uint;
    color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
    return *reinterpret_cast<float*> (&color_uint);
}

cv::Mat slMat2cvMat(sl::Mat& input) {
	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
		case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
		case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
		case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
		case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
		case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
		case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
		case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
		case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
		default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
