#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "rbflag_filter.h"

RBflagFilter::RBflagFilter(ros::NodeHandle& nh)
{
	this->R_H_Max = 185; // 203
	this->R_H_Min = 0;
	this->R_S_Max = 200;
	this->R_S_Min = 0;
	this->R_V_Max = 200;
	this->R_V_Min = 0;
	this->B_H_Max = 200;
	this->B_H_Min = 0;
	this->B_S_Max = 200;
	this->B_S_Min = 0;
	this->B_V_Max = 200;
	this->B_V_Min = 0;
	this->erosion_size = 1;

	//ros::init( NULL, NULL, "zed_ros_rb");
	//ros::NodeHandle nh("rb_flag");
	dsrv_ = new dynamic_reconfigure::Server<igvc_stereo::rbflag_filter_paramsConfig>(nh);
	dynamic_reconfigure::Server<igvc_stereo::rbflag_filter_paramsConfig>::CallbackType cb;
	cb = boost::bind(&RBflagFilter::configCallback, this, _1, _2);
    	dsrv_->setCallback(cb);
}

void RBflagFilter::configCallback(igvc_stereo::rbflag_filter_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    R_H_Max = config.groups.filter.R_H_Max_param; // 203
	R_H_Min = config.groups.filter.R_H_Min_param;
	R_S_Max = config.groups.filter.R_S_Max_param;
	R_S_Min = config.groups.filter.R_S_Min_param;
	R_V_Max = config.groups.filter.R_V_Max_param;
	R_V_Min = config.groups.filter.R_V_Min_param;
	B_H_Max = config.groups.filter.B_H_Max_param;
	B_H_Min = config.groups.filter.B_H_Min_param;
	B_S_Max = config.groups.filter.B_S_Max_param;
	B_S_Min = config.groups.filter.B_S_Min_param;
	B_V_Max = config.groups.filter.B_V_Max_param;
	B_V_Min = config.groups.filter.B_V_Min_param;
	erosion_size = config.groups.filter.erosion_size_param;

} // end configCallback()

/**
 * @brief WhitelineFilter::findLines This function finds the white lines in the
 * src_image
 * @param src_image the original image to find white lines in
 * @param rtrn_image the original image with cyan lines drawn where the white
 * lines were detected
 * @param lines a vector of start and end points for each line found
 *
 *  It Uses the following algorithm to find white lines:
 *     1. turn image into grayscale
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *     4. run it through an erosion filter
 *     5. run it through a Canny edge detector
 *     6. finally, take this processed image and find the lines using
 * Probabilistic Hough Transform HoughLinesP
 */
cv::Mat RBflagFilter::findRed(const cv::Mat& src_image)
{
    ros::spinOnce();
    // this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(this->original_image, this->hsv_image, CV_BGR2HSV);

    // Reduce resolution of image
    cv::GaussianBlur(this->hsv_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::inRange(this->blur_image, cv::Scalar(this->R_H_Min, this->R_S_Min, this->R_V_Min), cv::Scalar(this->R_H_Max, this->R_S_Max, this->R_V_Max), this->red_thresh_image);

    // Erode the image
/*    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(this->red_thresh_image, this->red_eroded_image, element);
*/
    //return eroded image;
//    return this->red_eroded_image;
	return this->red_thresh_image;
}

/**
 * @brief WhitelineFilter::findLines This function finds the white lines in the
 * src_image
 * @param src_image the original image to find white lines in
 * @param rtrn_image the original image with cyan lines drawn where the white
 * lines were detected
 * @param lines a vector of start and end points for each line found
 *
 *  It Uses the following algorithm to find white lines:
 *     1. turn image into grayscale
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *     4. run it through an erosion filter
 *     5. run it through a Canny edge detector
 *     6. finally, take this processed image and find the lines using
 * Probabilistic Hough Transform HoughLinesP
 */
cv::Mat RBflagFilter::findBlu(const cv::Mat& src_image)
{
    // this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(this->original_image, this->hsv_image, CV_BGR2HSV);

    // Reduce resolution of image
    cv::GaussianBlur(this->hsv_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::inRange(this->blur_image, cv::Scalar(this->B_H_Min, this->B_S_Min, this->B_V_Min), cv::Scalar(this->B_H_Max, this->B_S_Max, this->B_V_Max), this->blu_thresh_image);

    // Erode the image
    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(this->blu_thresh_image, this->blu_eroded_image, element);

    //return eroded image;
    return this->blu_eroded_image;
}

void RBflagFilter::filterControl()
{
    // Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow("ControlView", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Red Hue Max", "ControlView", &R_H_Max, 255);
    cv::createTrackbar("Red Hue Min", "ControlView", &R_H_Min, 255);
    cv::createTrackbar("Red Saturation Max", "ControlView", &R_S_Max, 255);
    cv::createTrackbar("Red Saturation Min", "ControlView", &R_H_Min, 255);
    cv::createTrackbar("Red Value Max", "ControlView", &R_V_Max, 255);
    cv::createTrackbar("Red Value Max", "ControlView", &R_V_Min, 255);
    cv::createTrackbar("Blue Hue Max", "ControlView", &B_H_Max, 255);
    cv::createTrackbar("Blue Hue Min", "ControlView", &B_H_Min, 255);
    cv::createTrackbar("Blue Saturation Max", "ControlView", &B_S_Max, 255);
    cv::createTrackbar("Blue Saturation Min", "ControlView", &B_H_Min, 255);
    cv::createTrackbar("Blue Value Max", "ControlView", &B_V_Max, 255);
    cv::createTrackbar("Blue Value Max", "ControlView", &B_V_Min, 255);
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayOriginal()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->original_image, disImage, cv::Size(400, 300));
        cv::imshow("Original Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}


/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Blurred image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayBlurred()
{
    try {
        cv::Mat disImage;
        cv::resize(this->blur_image, disImage, cv::Size(400, 300));
        cv::imshow("Blurred Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayRedThreshold()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->red_thresh_image, disImage, cv::Size(400, 300));
        cv::imshow("Red Threshold Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayBluThreshold()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->blu_thresh_image, disImage, cv::Size(400, 300));
        cv::imshow("Blue Threshold Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Eroded image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayRedEroded()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->red_eroded_image, disImage, cv::Size(400, 300));
        cv::imshow("Red Eroded Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Eroded image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void RBflagFilter::displayBluEroded()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->blu_eroded_image, disImage, cv::Size(400, 300));
        cv::imshow("Blue Eroded Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}



