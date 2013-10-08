/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         visualization.cpp

 \author       Daniel Kappler
 \date         April 15, 2013

 *********************************************************************/

#ifndef IMAGE_TRANSFORMATIONS_H_
#define IMAGE_TRANSFORMATIONS_H_

#include <grasp_template/template_heightmap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace deep_learning {

void Render_image(const cv::Mat &image_full,const cv::Mat &image_map,
		cv::Mat &image_result);

void Render_image(const cv::Mat &image_full,const cv::Mat &image_map,
		cv::Mat &image_result, unsigned char channel);

void Render_image(const grasp_template::TemplateHeightmap &heightmap,
		double min_z, double max_z, cv::Mat &image_full, cv::Mat &image_map);

uint8_t Compute_depth_value(float z_depth, float min_z, float max_z);

void Normalize_image(const cv::Mat &image, double &angle_result,
		bool &flip_result);

cv::Mat Transform_image(const cv::Mat &image, double angle, bool flip,
		unsigned int size);

cv::Mat Rotate_image(const cv::Mat &image, const cv::Point2f &center,
		double angle);

bool Mean_upper(const cv::Mat &image, const cv::Point2f &center);

bool Compute_mean(const cv::Mat &image, cv::Point2f &center);

}

#endif /* VISUALIZATION */
