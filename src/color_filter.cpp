#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "color_filter.h"

ColorFilter::ColorFilter()
{
	dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig>::CallbackType cb;
	cb = boost::bind(&ColorFilter::configCallback, this, _1, _2);
	dr_srv_.setCallback(cb);
}

void ColorFilter::configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	this->max_delt = config.max_delt_param;
	this->min_val = config.min_val_param;

	this->R_H_Max = config.R_H_Max_param; // 203
	this->R_H_Min = config.R_H_Min_param;
	this->R_S_Max = config.R_S_Max_param;
	this->R_S_Min = config.R_S_Min_param;
	this->R_V_Max = config.R_V_Max_param;
	this->R_V_Min = config.R_V_Min_param;
	this->B_H_Max = config.B_H_Max_param;
	this->B_H_Min = config.B_H_Min_param;
	this->B_S_Max = config.B_S_Max_param;
	this->B_S_Min = config.B_S_Min_param;
	this->B_V_Max = config.B_V_Max_param;
	this->B_V_Min = config.B_V_Min_param;

	this->thresh_val=config.thresh_val_param;
	this->erosion_size=config.erosion_size_param;
	this->h_rho=config.h_rho_param;
	this->h_theta=config.h_theta_param;
	this->h_thresh=config.h_thresh_param;
	this->h_minLineLen=config.h_minLineLen_param;
	this->h_maxLineGap=config.h_maxLineGap_param;
	this->lower_limit=config.lowerLimit_param;
	this->upper_limit=config.upperLimit_param;
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
cv::gpu::GpuMat ColorFilter::findLines(const cv::Mat& src_image)
{
    this->original_image.upload(src_image);
    // Convert the BGR image to Gray scale
    cv::gpu::cvtColor(this->original_image, this->gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::gpu::GaussianBlur(this->gray_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0,
        cv::BORDER_DEFAULT);

    // Threshold the image
    cv::gpu::threshold(this->blur_image, this->thresh_image, this->thresh_val, 1,
        cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::gpu::erode(this->thresh_image, this->eroded_image, element);

    return this->eroded_image;
}


int ColorFilter::rgb2hsv(uint32_t rgb){
	uint8_t B = (rgb >> 16) & 0x0000ff;
	uint8_t G = (rgb >> 8) & 0x0000ff;
	uint8_t R = (rgb) & 0x0000ff;
	double r = (double)R/255;
	double g = (double)G/255;
	double b = (double)B/255;
	//printf("r: %d, g: %d, b: %d\n", R, G, B);
	double min, max, delta, h, s, v;
	int H, S, V;

	min = r < g ? r : g;
	min = min < b ? min : b;
	
	max = r > g ? r : g;
	max = max > b ? max : b;

	v = max;
	delta = max - min;
	if (max == 0) return 0;
	if (delta == 0) ++delta; 
	s = delta/max;

	if (r == max) h = (g - b)/delta;
	else if (g == max) h = 2.0 + (b - r)/delta;
	else if (b == max) h = 4.0 + (r - g)/delta;

	h *= 60;

	if (h < 0) h = h + 360.0;
	h = h*255.0/360.0;
	H = int(h); S = int(s*255); V = int(v*255);
	//printf("h: %d, s: %d, v: %d\n", H, S, V);

	return (H << 16)|(S << 8)|(V);
}

bool ColorFilter::checkWhite(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	uint8_t B = (color_uint >> 16) & 0x0000ff;
	uint8_t G = (color_uint >> 8) & 0x0000ff;
	uint8_t R = (color_uint) & 0x0000ff;
	int max, min, mid, delt;

	max = B > G ? B : G;
	max = max > R ? max : R;

	min = B < G ? B : G;
	min = min < R ? min : R;
	
	if (B >= G && B <= R) mid = B;
	else if (G <= B && G >= R) mid = G;
	else mid = R;

	delt = (max-mid) > (mid-min) ? (max-mid) : (mid-min);
	return (delt <= this->max_delt && mid > this->min_val);
}

bool ColorFilter::checkRed(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	int hsv = rgb2hsv(uint32_t(color_uint));
	int h, s, v;
	h = (hsv >> 16) & 0x0000ff;
	s = (hsv >> 8) & 0x0000ff;
	v = (hsv) & 0x0000ff;

	return ((h < this->R_H_Max || h > this->R_H_Min) && s > R_S_Min && v > R_V_Min);
}

bool ColorFilter::checkBlu(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	int hsv = rgb2hsv(uint32_t(color_uint));
	int h, s, v;
	h = (hsv >> 16) & 0x0000ff;
	s = (hsv >> 8) & 0x0000ff;
	v = (hsv) & 0x0000ff;

	return (h < this->B_H_Max && h > this->B_H_Min && s > B_S_Min && v > B_V_Min);
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayOriginal()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->original_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
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
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayThreshold()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->thresh_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
        cv::imshow("Threshold Image", disImage);
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
void ColorFilter::displayEroded()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->eroded_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
        cv::imshow("Eroded Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Canny Edge image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayCanny()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->canny_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
        cv::imshow("Canny Edge Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}


