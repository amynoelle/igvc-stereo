///////////////////////////////////////////////////////////////////////////
// Main file for publishing point clouds with the ZED Camera on the 
// Jetson TK1 board. This file is responsible for initilizing the ZED 
// camera, ROS node, and filter classes. 
// The main purpose of this file is to filter the ZED disparity images for 
// white lines and red/blue flags, and then publish the resulting point 
// cloud to ROS topics for use by the IGVC navigation.
///////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

// 2016 Pipeline includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "color_filter.h"

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cuda.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <sstream>
 
void log2file( const std::string &text )
{
    std::ofstream log_file(
        "log_file.txt", std::ios_base::out | std::ios_base::app );
    log_file << text;
}

using namespace sl::zed;
using namespace std;

Camera* zed;
SENSING_MODE dm_type = STANDARD;

int main(int argc, char** argv) {
    /*
    Initialize ZED Camera
    */
    ROS_INFO("JETSON TK1 is initializing ZED\n");
    zed = new Camera(VGA);
    
    sl::zed::InitParams params;
    params.mode = PERFORMANCE;
    params.unit = METER;
    params.coordinate = RIGHT_HANDED;
    params.verbose = true;
    ERRCODE err = zed->init(params);
    ROS_INFO("JETSON TK1 error code for params is: %s", errcode2str(err).c_str());
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }
    ROS_INFO("JETSON TK1 is Initializing ROS\n");
    bool pub_images=false;
    ros::init(argc, argv, "zed_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_flags("rb_flag");
    nh.param("/pub_images", pub_images, false);
    
    std::string white_cloud_topic = "point_cloud/white_cloud";
    std::string point_cloud_frame_id = "/zed_current_frame";
    ros::Publisher white_publisher_cloud = nh.advertise<sensor_msgs::PointCloud2> (white_cloud_topic, 2);
    
    ROS_INFO("JETSON TK1 pub_image value is (0 means do not publish images): %d",pub_images);
    image_transport::Publisher image_publisher;
    image_transport::Publisher white_publisher;
    if (pub_images){
        image_transport::ImageTransport it_image(nh);
        image_publisher = it_image.advertise("camera/image", 1);
        image_transport::ImageTransport it_white(nh);
	    white_publisher = it_white.advertise("camera/white", 1);
    }
    ColorFilter color_filter;

    ROS_INFO("JETSON TK1 is entering main loop\n");
    // The benchmark timing is based on git commit: 358bcfa4d3bc14d0e72ee38c2ca35b9e420016f6 master branch
    //log2file("top,afgrab,afRmeasure,afRImage,afFilter,afDwnImg,b4Forloop,afForloop,b4Sleep, CLOCKS_PER_SEC,cpu_cloud.rows,cpu_cloud.cols,cpu_cloud.step0,cpu_cloud.step1,cpu_cloud.elemSize, cpu_cloud.elemSize1,cpu_cloud.depth,cpu_cloud.channels,sizeof\n"); 
    ros::Rate loop_rate(20);
    while (nh.ok()) {
        std::ostringstream outStream;
        std::ostringstream sizesLog;
	    //outStream << clock() << ","; // TIMER: top
        pcl::PointXYZRGBA point;
        pcl::PointCloud<pcl::PointXYZRGBA> white_cloud;
        white_cloud.clear();
        cv::Mat cloud_image;
	    
        if (!zed->grab(dm_type)) { // grabs a new image
	        //outStream << clock() << ","; // TIMER: afgrab 20.16ms/47% of time
		    //Get image from ZED using the gpu buffer
		    cv::Mat cpu_cloud = slMat2cvMat(zed->retrieveMeasure(MEASURE::XYZBGRA));
		    //sizesLog << cpu_cloud.rows    << "," << cpu_cloud.cols       << ",";
		    //sizesLog << cpu_cloud.step[0]    << ","<< cpu_cloud.step[1]    << "," << cpu_cloud.elemSize() << "," << cpu_cloud.elemSize1() << ",";
		    //sizesLog << cpu_cloud.depth() << "," << cpu_cloud.channels() << "," << sizeof(cpu_cloud)<< ",";
	        //outStream << clock() << ","; // TIMER: afRmeasure 6ms/13.8% of time
            cv::Mat  host_image = slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT));
	        //outStream << clock() << ","; // TIMER: afRImage 2.6ms/6.1% of time
		
		    //Filter the image for white lines
		    cv::gpu::GpuMat cv_filteredImage = color_filter.findLines(host_image);
	        //outStream << clock() << ","; // TIMER: afFilter 21.1/9.1% of time
		    cv_filteredImage.download(cloud_image); //move image to cpu memory
	        //outStream << clock() << ","; // TIMER: afDwnImg 0.65ms/1.6% of time
		    //sizesLog << cloud_image.rows    << "," << cloud_image.cols       << ",";
		    //sizesLog << cloud_image.step    << "," << cloud_image.elemSize() << "," << cloud_image.elemSize1() << ",";
		    //sizesLog << cloud_image.depth() << "," << cloud_image.channels() << "," << sizeof(cloud_image)<< "\n";
		    
		    if(pub_images){
	  	        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", host_image).toImageMsg();
                image_publisher.publish(img_msg);
		        sensor_msgs::ImagePtr fltrd_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cloud_image).toImageMsg();
                white_publisher.publish(fltrd_msg);
            }
		    //imwrite("/home/ubuntu/image.jpg", cloud_image);
	        //outStream << clock() << ","; // TIMER: b4Forloop 3.1ms/7.2% of time 		
        	//Iterate through points in cloud

            //TODO This could be faster if we used the GPU to produce a list of pixels of interest and iterated over just those pixels as opposed to
            // iterating over every pixel in the image.
		    for (int i = 0; i < cloud_image.rows; i++) { 
		        for (int j = 0; j < cloud_image.cols;j++) {		        
                    int wl_point = cloud_image.at<int>(i,j);
		            if (wl_point > 10){
		                cv::Vec4f cloud_pt = cpu_cloud.at<cv::Vec4f>(i,j*4);
                        point.x = -cloud_pt.val[2];
                    	point.y = -cloud_pt.val[0];
                    	point.z = cloud_pt.val[1];
                    	point.rgb = cloud_pt.val[3];
                        white_cloud.push_back(point);
		            }
	            }
	        }		    
	        //outStream << clock() << ","; // TIMER: afForloop 0.91ms/2.2% of time		

		    if (white_cloud.height == 0){
			    white_cloud.push_back(pcl::PointXYZRGBA());
		    }		    
            sensor_msgs::PointCloud2 output_white;
		    pcl::toROSMsg(white_cloud, output_white); // Convert the point cloud to a ROS message
       		output_white.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
       		output_white.header.stamp = ros::Time::now();
       		output_white.is_bigendian = false;
       		output_white.is_dense = false;
       		white_publisher_cloud.publish(output_white);
		    white_cloud.clear();
		    //Spin once updates dynamic_reconfigure values
	        //outStream << clock() << ","<< CLOCKS_PER_SEC << ","; // TIMER: b4Sleep 0.29ms//0.7% of time	
            //log2file(outStream.str());
            //log2file(sizesLog.str());
		    ros::spinOnce();
		    loop_rate.sleep(); 
	    }
   } // Average time for the big while loop is: 43.1 ms about 23hz b4 change rate from 30hz to 20hz.
   // At full speed (5mph)the robot moves about 11cm in the time it takes to complete this loop.
    delete zed;
    return 0;
}
