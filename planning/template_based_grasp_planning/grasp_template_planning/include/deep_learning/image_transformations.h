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
cv::Mat Render_image_4channel(
		const grasp_template::TemplateHeightmap &heightmap);
cv::Mat Render_image_solid(const grasp_template::TemplateHeightmap &heightmap);
cv::Mat Render_image_table(const grasp_template::TemplateHeightmap &heightmap);
cv::Mat Render_image_fog(const grasp_template::TemplateHeightmap &heightmap);
cv::Mat Render_image_dontcare(
		const grasp_template::TemplateHeightmap &heightmap);

cv::Mat Normalize_image(const cv::Mat image,int size);
cv::Mat Rotate_image(const cv::Mat image, const cv::Point2f &center,double angle);
bool Mean_upper(const cv::Mat image, const cv::Point2f &center);
bool Compute_mean(const cv::Mat image, cv::Point2f &center);
}

#endif /* VISUALIZATION */
