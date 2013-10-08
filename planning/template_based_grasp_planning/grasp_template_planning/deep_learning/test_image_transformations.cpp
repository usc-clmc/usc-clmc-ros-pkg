/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         main-visualization.cpp

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#include <iostream>
#include <opencv2/opencv.hpp>
#include <deep_learning/image_transformations.h>

using namespace deep_learning;

int main(int argc, char** argv) {
	std::string base_path = "/home/dkappler/data-storage/image-tests/";
	cv::Mat image_upper_right = cv::imread(base_path + "up_right.bmp",
			CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat image_upper_left = cv::imread(base_path + "up_left.bmp",
			CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat image_down_right = cv::imread(base_path + "down_right.bmp",
			CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat image_down_left = cv::imread(base_path + "down_left.bmp",
			CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat image_down_left_flip = cv::imread(base_path + "down_left_flip.bmp",
			CV_LOAD_IMAGE_GRAYSCALE);

	std::cout << " begin processing " << std::endl;

	const unsigned int size = 32;
	double angle = 0.0;
	bool flip = false;
	Normalize_image(image_upper_right,  angle, flip);
	cv::imwrite((base_path + "up_right_norm.bmp").c_str(),
			Transform_image(image_upper_right, angle, flip, size));
	Normalize_image(image_upper_left,  angle, flip);
	cv::imwrite((base_path + "up_left_norm.bmp").c_str(),
			Transform_image(image_upper_left, angle, flip, size));

	Normalize_image(image_down_right,  angle, flip);
	cv::imwrite((base_path + "down_right_norm.bmp").c_str(),
			Transform_image(image_down_right, angle, flip, size));

	Normalize_image(image_down_left,  angle, flip);
	cv::imwrite((base_path + "down_left_norm.bmp").c_str(),
			Transform_image(image_down_left, angle, flip, size));

	Normalize_image(image_down_left_flip,  angle, flip);
	cv::imwrite((base_path + "down_left_flip_norm.bmp").c_str(),
			Transform_image(image_down_left_flip, angle, flip, size));

	std::cout << "done " << std::endl;
	return 0;
}
