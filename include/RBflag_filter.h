#ifndef WHITELINE_FILTER_H
#define WHITELINE_FILTER_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <igvc_stereo/rbflag_filter_paramsConfig.h>

class RBflagFilter {
public:
    RBflagFilter();

    //! Callback function for dynamic reconfigure server.
    void configCallback(igvc_stereo::line_filter_paramsConfig &config, uint32_t level);
    void filterControl();
    cv::Mat findRed(const cv::Mat& src_image);
    cv::Mat findBlu(const cv::Mat& src_image);
    void displayOriginal();
    void displayBlurred();
    void displayRedThreshold();
    void displayBluThreshold();
    void displayRedEroded();
    void displayBluEroded();

private:
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<igvc_stereo::rbflag_filter_paramsConfig> dr_srv_;
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
    cv::Mat original_image; // The original source image
    cv::Mat hsv_image; // The image after it is converted to grayscale
    cv::Mat blur_image; // The image after it a blur effect is applied
    cv::Mat red_thresh_image; // The image after the colors are categorized by defined threshold values
    cv::Mat blu_thresh_image; // The image after the colors are categorized by defined threshold values
    cv::Mat red_eroded_image; // The image after an erosion filter is applied
    cv::Mat blu_eroded_image; // The image after an erosion filter is applied
    int lower_limit; // The image after all detected white lines are drawn in cyan color
    int upper_limit; // The image after all detected white lines are drawn in cyan color
};

#endif // WHITELINE_FILTER_H


