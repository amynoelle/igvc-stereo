///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
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
// TODO publish image in ROS

// error: (-27) create() called for the missing output array in function create
// error: (-213) Unknown/unsupported array type in function getMat_

/***************************************************************************************************
 ** This sample demonstrates how to grab images and depth map with the ZED SDK                    **
 ** and apply the result in a 3D view "point cloud style" with OpenGL /freeGLUT                   **
 ** Some of the functions of the ZED SDK are linked with a key press event		                  **
 ***************************************************************************************************/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenGL includes
#include <GL/glew.h>
#include <GL/freeglut.h> 
#include <GL/gl.h>
#include <GL/glut.h>

// ZED includes
#include <sl/Camera.hpp>
#include <sl/Core.hpp>
#include <sl/defines.hpp>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sl/Core.hpp>
//#include <sl/defines.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

//// Using std and sl namespaces
using namespace std;
using namespace sl;

//// Create ZED object (camera, callback, images)
sl::Camera zed;
sl::Mat point_cloud, left_image;

void close();
cv::Mat slMat2cvMat(sl::Mat& input);

int main(int argc, char **argv) {
    ros::init(argc, argv, "zed_ros_node");
    ros::NodeHandle nh;
    bool pub_images=false;
    nh.param("/pub_images", pub_images, false);
    ROS_INFO("JETSON TX2 pub_images is: %d",pub_images);
    image_transport::Publisher image_publisher;
    image_transport::Publisher white_publisher;
    if (pub_images){
        image_transport::ImageTransport it_image(nh);
        image_publisher = it_image.advertise("camera/image", 1);
        image_transport::ImageTransport it_white(nh);
	    white_publisher = it_white.advertise("camera/white", 1);
    }    
    std::string pcl2_cloud_topic = "point_cloud/vision_cloud";
    std::string point_cloud_frame_id = "/zed_current_frame";
    ros::Publisher pcl_publisher = nh.advertise<sensor_msgs::PointCloud2> (pcl2_cloud_topic, 2);

    // Setup configuration parameters for the ZED
    InitParameters initParameters;
    
    if (argc == 2) initParameters.svo_input_filename = argv[1];
    initParameters.camera_resolution = sl::RESOLUTION_HD720;
    initParameters.depth_mode = sl::DEPTH_MODE_PERFORMANCE; //need quite a powerful graphic card in QUALITY
    initParameters.coordinate_units = sl::UNIT_METER; // set meter as the OpenGL world will be in meters
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed

    // Open the ZED
    ERROR_CODE err = zed.open(initParameters);
    if (err != SUCCESS) {
        ROS_INFO("JETSON TX2 error code on zed open is: %s", sl::errorCode2str(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }
    ROS_INFO("JETSON TX2 initalized the ZED");

    ROS_INFO("JETSON TX2 is entering main loop");
    ros::Rate loop_rate(30);
    while (nh.ok()) {
        try{
            pcl::PointXYZRGBA point;
            pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud;
            if (zed.grab() == SUCCESS) {
                // Get depth as a displayable image (8bits) and display it with OpenCV
                zed.retrieveImage(left_image, sl::VIEW_LEFT); // For display purpose ONLY. To get real world depth values, use retrieveMeasure(mat, sl::MEASURE_DEPTH)
                //cv::imshow("Image", cv::Mat(left_image.getHeight(), left_image.getWidth(), CV_8UC4, left_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
                //cv::waitKey(15);
                if(pub_images){
	      	        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgra8", slMat2cvMat(left_image)).toImageMsg();
                    image_publisher.publish(img_msg);
		            //sensor_msgs::ImagePtr fltrd_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cloud_image).toImageMsg();
                    //white_publisher.publish(fltrd_msg);
                }
                // Get XYZRGBA point cloud on GPU and send it to OpenGL
                zed.retrieveMeasure(point_cloud, sl::MEASURE_XYZRGBA, sl::MEM_GPU); // Actual metrics values
                
		        for (size_t i = 0; i < point_cloud.getWidth(); i+=2) {
		            for (size_t j = 0; j < point_cloud.getHeight();j+=2) {		        
		                    sl::float4 cloud_pt;
		                    point_cloud.getValue(i,j,&cloud_pt, sl::MEM_GPU);
                            point.x = cloud_pt.x;
                        	point.y = cloud_pt.y;
                        	point.z = cloud_pt.z;
                        	point.rgb = cloud_pt.w;
                            pcl_cloud.push_back(point);
		                }
	                }
                
            }
            else sl::sleep_ms(1);
            
		    if (pcl_cloud.height == 0){
			    pcl_cloud.push_back(pcl::PointXYZRGBA());
		    }		    
            sensor_msgs::PointCloud2 output_pcl2;
		    pcl::toROSMsg(pcl_cloud, output_pcl2); // Convert the point cloud to a ROS message
       		output_pcl2.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
       		output_pcl2.header.stamp = ros::Time::now();
       		output_pcl2.is_bigendian = false;
       		output_pcl2.is_dense = false;
       		pcl_publisher.publish(output_pcl2);
		    pcl_cloud.clear();
        }
        catch (const std::exception& e){
                ROS_INFO("Exception caught grabbing from zed: %s", e.what() );                
        }
        ros::spinOnce();
        loop_rate.sleep();
    } 
    close();
    return 0;
}

/**
 *  This function frees and close the ZED, its callback(thread) and the viewer
 **/
void close() {
    // free buffer and close ZED
    left_image.free(MEM_CPU);
    point_cloud.free(MEM_GPU);
    zed.close();
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
