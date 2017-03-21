#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <igvc_stereo/color_filter_paramsConfig.h>

class ColorFilter {
public:
    ColorFilter();

    //! Callback function for dynamic reconfigure server.
    void configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level);
    cv::Mat findLines(const cv::Mat& src_image);
    cv::Mat findRed(const cv::Mat& src_image);
    cv::Mat findBlu(const cv::Mat& src_image);

private:
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig> dr_srv_;
    void findPointsOnLines(const cv::Mat& cImage);
    cv::vector<cv::Vec4i> lines;
    std::vector<cv::Point2i> pixels;
    int R_H_Max = 185; // 203
    int R_H_Min = 0;
    int R_S_Max = 200;
    int R_S_Min = 0;
    int R_V_Max = 200;
    int R_V_Min = 0;
    int B_H_Max = 200;
    int B_H_Min = 0;
    int B_S_Max = 200;
    int B_S_Min = 0;
    int B_V_Max = 200;
    int B_V_Min = 0;
    int thresh_val; // The threshold value used to identify white in the image
    int erosion_size; // The  size of our kernel to erode the image with
    int h_rho; // Distance resolution of the accumulator in pixels (hough transform)
    int h_theta; // Angle resolution of the accumulator in radians (hough transform)
    int h_thresh; // Accumulator threshold parameter. Only those lines are returned that get enough votes (hough transform)
    int h_minLineLen; // Line segments shorter than that are rejected (hough transform)
    int h_maxLineGap; // Maximum allowed gap between points on the same line to link them (hough transform)
    cv::Mat original_image; // The original source image
    cv::Mat gray_image; // The image after it is converted to grayscale
    cv::Mat hsv_image;	// 
    cv::Mat blur_image; // The image after it a blur effect is applied
    cv::Mat thresh_image; // The image after the colors are categorized by defined threshold values
    cv::Mat eroded_image; // The image after an erosion filter is applied
    cv::Mat canny_image; // The image after canny edge detection is complete
    cv::Mat hough_image; // The image with the hugh line transform is applied
    cv::Mat cyan_image; // The image after all detected white lines are drawn in cyan color
    cv::Mat blu_thresh_image;
    cv::Mat red_thresh_image;
    int lower_limit; // The image after all detected white lines are drawn in cyan color
    int upper_limit; // The image after all detected white lines are drawn in cyan color
};

#endif // WHITELINE_FILTER_H


