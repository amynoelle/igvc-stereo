#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "color_filter.h"

ColorFilter::ColorFilter()
{
	this->thresh_val = 185; // 203
	this->erosion_size = 1; // 2
	this->h_rho = 1; // 1
	this->h_theta = 180; // 180
	this->h_thresh = 30; // 40
	this->h_minLineLen = 21; // 20
	this->h_maxLineGap = 20; // 30
	this->lower_limit=118;
	this->upper_limit=250;
	this->R_H_Max = 80; // 203
	this->R_H_Min = 0;
	this->R_S_Max = 255;
	this->R_S_Min = 89;
	this->R_V_Max = 255;
	this->R_V_Min = 50;
	this->B_H_Max = 130;
	this->B_H_Min = 70;
	this->B_S_Max = 255;
	this->B_S_Min = 170;
	this->B_V_Max = 255;
	this->B_V_Min = 120;

	dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig>::CallbackType cb;
	cb = boost::bind(&ColorFilter::configCallback, this, _1, _2);
	dr_srv_.setCallback(cb);
}

void ColorFilter::configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
    thresh_val=config.groups.filter.thresh_val_param;
    erosion_size=config.groups.filter.erosion_size_param;
    h_rho=config.groups.hough.h_rho_param;
    h_theta=config.groups.hough.h_theta_param;
    h_thresh=config.groups.hough.h_thresh_param;
    h_minLineLen=config.groups.hough.h_minLineLen_param;
    h_maxLineGap=config.groups.hough.h_maxLineGap_param;
    lower_limit=config.groups.hough.lowerLimit_param;
    upper_limit=config.groups.hough.upperLimit_param;

	R_H_Max = config.groups.flags.R_H_Max_param; // 203
	R_H_Min = config.groups.flags.R_H_Min_param;
	R_S_Max = config.groups.flags.R_S_Max_param;
	R_S_Min = config.groups.flags.R_S_Min_param;
	R_V_Max = config.groups.flags.R_V_Max_param;
	R_V_Min = config.groups.flags.R_V_Min_param;
	B_H_Max = config.groups.flags.B_H_Max_param;
	B_H_Min = config.groups.flags.B_H_Min_param;
	B_S_Max = config.groups.flags.B_S_Max_param;
	B_S_Min = config.groups.flags.B_S_Min_param;
	B_V_Max = config.groups.flags.B_V_Max_param;
	B_V_Min = config.groups.flags.B_V_Min_param;
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
cv::Mat ColorFilter::findLines(const cv::Mat& src_image)
{
    this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(src_image, this->gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::GaussianBlur(this->gray_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0,
        cv::BORDER_DEFAULT);

    // Threshold the image
    cv::threshold(this->blur_image, this->thresh_image, this->thresh_val, 1,
        cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(this->thresh_image, this->eroded_image, element);

    // Canny edge detection
    cv::Canny(this->eroded_image, this->canny_image, 50, 250, 3);

    // Prevent any divide by zero errors
    if (this->h_rho <= 0) {
        this->h_rho = 1;
    }
    if (this->h_theta <= 0) {
        this->h_theta = 1;
    }
    if (this->h_thresh <= 0) {
        this->h_thresh = 1;
    }

    // Find the Hough lines
    cv::HoughLinesP(this->canny_image, lines, this->h_rho,
        (CV_PI / this->h_theta), this->h_thresh, this->h_minLineLen,
        this->h_maxLineGap);
    this->hough_image = cv::Mat::zeros(canny_image.size(), CV_8UC1);
    this->cyan_image = src_image.clone();

    // Draw the Hough lines on the image
    for (int i = 0; i < lines.size(); i++) {
        line(this->hough_image, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), 255, 3, 8);
        line(this->cyan_image, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 0), 5, 8);
    }
    for (int i=0; i < hough_image.rows;i++){
        for (int j=0; j<hough_image.cols;j++){
            if (i <= this->upper_limit){
                hough_image.at<unsigned char>(i,j)=0;
            }
            else if (i >= this->lower_limit){
                hough_image.at<unsigned char>(i,j)=0;
            }
        }
    }

    //return hough_image;
    return this->cyan_image;
}



/**
 * @brief whiteline_filter::findPointsOnLines. Finds the x,y coordinates of each
 * point on line defined by an start and end point.
 * @param cImage The image that lines exist in
 * @param lines A list of lines defined by start and end points
 * @param pixels returns a list of pixels that are on the lines.
 */
void ColorFilter::findPointsOnLines(const cv::Mat& cImage)
{
    cv::Point pt1;
    cv::Point pt2;

    for (int i = 0; i < lines.size(); i++) {
        pt1.x = lines[i][0];
        pt1.y = lines[i][1];
        pt2.x = lines[i][2];
        pt2.y = lines[i][3];
        cv::LineIterator it(cImage, pt1, pt2, 8);
        for (int j = 0; j < it.count; j++, ++it) {
            pixels.push_back(cv::Point2i(it.pos().x, it.pos().y));
        }
    }
}

/**
 * @brief RBflagFilter::findRed This function finds the red flags in the src_image
 * @param src_image the original image to find red in
 * @param rtrn_image a threshold image where white is the flags found
 *
 *  It Uses the following algorithm to find red flags:
 *     1. convert image to HSV
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *
 */
cv::Mat ColorFilter::findRed(const cv::Mat& src_image)
{
	this->red_image = src_image;
	// Convert the BGR image to Gray scale
	cvtColor(this->red_image, this->hsv_image_red, CV_BGR2HSV);

	// Reduce resolution of image
	cv::GaussianBlur(this->hsv_image_red, this->blur_image_red, cv::Size(7, 7), 0.0, 0.0, cv::BORDER_DEFAULT);

	// Threshold the image
	cv::inRange(this->blur_image_red, cv::Scalar(this->R_H_Min, this->R_S_Min, this->R_V_Min), cv::Scalar(this->R_H_Max, this->R_S_Max, this->R_V_Max), this->red_thresh_image);

	// Erode the image
/*    	cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    	cv::erode(this->red_thresh_image, this->red_eroded_image, element);
*/
	//return eroded image;
	return this->red_thresh_image;
}

/**
 * @brief RBflagFilter::findBlu This function finds the blue flags in the src_image
 * @param src_image the original image to find blue in
 * @param rtrn_image a threshold image where white is the flags found
 *
 *  It Uses the following algorithm to find blue flags:
 *     1. convert image to HSV
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *
 */
cv::Mat ColorFilter::findBlu(const cv::Mat& src_image)
{
    this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(this->original_image, this->hsv_image_blu, CV_BGR2HSV);

    // Reduce resolution of image
    cv::GaussianBlur(this->hsv_image_blu, this->blur_image_blu, cv::Size(7, 7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::inRange(this->blur_image_blu, cv::Scalar(this->B_H_Min, this->B_S_Min, this->B_V_Min), cv::Scalar(this->B_H_Max, this->B_S_Max, this->B_V_Max), this->blu_thresh_image);

    // Erode the image
/*    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(this->blu_thresh_image, this->blu_eroded_image, element);*/

    //return eroded image;
    return this->blu_thresh_image;
}

/**
 * @brief RBflagFilter::displayRedBlurred Use OpenCV imShow to display the
 *Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayOriginal()
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
 * @brief RBflagFilter::displayRedThreshold Use OpenCV imShow to display the
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayRedThreshold()
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
 * @brief RBflagFilter::displayBluThreshold Use OpenCV imShow to display the
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayBluThreshold()
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


