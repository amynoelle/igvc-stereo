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

    int rgb2hsv(uint32_t rgb);
    bool checkRed(float rgb);
    bool checkBlu(float rgb);
    bool checkWhite(float rgb);


private:
    //! Dynamic reconfigure server.
    dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig> dr_srv_;
    int max_delt;
    int min_val;
    int R_H_Max; // 203
    int R_H_Min;
    int R_S_Max;
    int R_S_Min;
    int R_V_Max;
    int R_V_Min;
    int B_H_Max;
    int B_H_Min;
    int B_S_Max;
    int B_S_Min;
    int B_V_Max;
    int B_V_Min;
};

#endif // WHITELINE_FILTER_H


