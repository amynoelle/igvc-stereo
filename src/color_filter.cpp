#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "color_filter.h"

ColorFilter::ColorFilter()
{
	this->max_delt = 15;
	this->min_val = 150;
	this->R_H_Max = 40; // 203
	this->R_H_Min = 230;
	this->R_S_Max = 255;
	this->R_S_Min = 190;
	this->R_V_Max = 255;
	this->R_V_Min = 102;
	this->B_H_Max = 200;
	this->B_H_Min = 150;
	this->B_S_Max = 255;
	this->B_S_Min = 150;
	this->B_V_Max = 255;
	this->B_V_Min = 50;

	dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig>::CallbackType cb;
	cb = boost::bind(&ColorFilter::configCallback, this, _1, _2);
	dr_srv_.setCallback(cb);
}

void ColorFilter::configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	max_delt = config.groups.lines.max_delt_param;
	min_val = config.groups.lines.min_val_param;

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


