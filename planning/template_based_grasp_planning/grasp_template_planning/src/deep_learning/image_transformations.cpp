/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         howto_extract_templates.cpp

 \author       Alexander Herzog
 \date         April 15, 2013

 *********************************************************************/

#include <deep_learning/image_transformations.h>

#include <cmath>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <usc_utilities/file_io.h>
#include <visualization_msgs/Marker.h>

#include <grasp_template/dismatch_measure.h>
#include <grasp_template/heightmap_sampling.h>
#include <grasp_template_planning/GraspLog.h>

namespace deep_learning {

void Normalize_image(const cv::Mat &image, double &angle_result,
		bool &flip_result) {

	assert(image.rows == image.cols);

	angle_result = 0;
	flip_result = false;

	// the mean position should be on the center horizontal line
	cv::Point2f image_center;
	image_center.x = image.size().width / 2;
	image_center.y = image.size().height / 2;

	// we compute the mean of the image of all points which are not black
	// assuming that the data is uint8_t
	cv::Point2f center;
	if (!Compute_mean(image, center)) {
		return;
	}

	float direction_cols = center.x - image_center.x;
	float direction_rows = center.y - image_center.y;

	// the direction has to be normalized for the angle computation
	// angle is in negative horizont
	float direction_norm = direction_rows
			/ sqrt(
					direction_cols * direction_cols
							+ direction_rows * direction_rows);
	angle_result = acos(-direction_norm) * 180.0f / (float) CV_PI;

	// if the it is below then the direction has to change
	if (center.x > image_center.x) {
		angle_result *= -1;
	}

	cv::Mat result = Rotate_image(image, image_center, angle_result);

	// debugging
//	int block_size = 3;
//	for(int rows = -block_size; rows < block_size; ++rows){
//		for(int cols = -block_size; cols < block_size; ++cols){
//			result.at<uint8_t>(image_center.y+rows,image_center.x+cols) = 155;
//		}
//	}
	// debugging

	if (Mean_upper(result, image_center)) {
		//cv::flip(result, result, 0);
		flip_result = true;
	}
	//result.resize(size,size);
	//return result;
}

cv::Mat Rotate_image(const cv::Mat &image, const cv::Point2f &center,
		double angle) {
	std::cout << "rot center " << center << std::endl;
	std::cout << "rot angle " << angle << std::endl;
	cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
	cv::Mat result;
	cv::warpAffine(image, result, rot_mat, cv::Size(image.cols, image.rows),
			CV_INTER_LINEAR);
	result.resize(image.cols, image.rows);
	return result;
}

cv::Mat Transform_image(const cv::Mat &image, double angle, bool flip,
		unsigned int size) {
	if (sqrt(2 * pow(size / 2, 2)) > (image.rows / 2)) {
		std::cout
				<< "size has to square has to fit inside the image for any rotation"
				<< "image rows " << image.rows << " cols " << image.cols
				<< "size " << size
				<< std::endl;
		return cv::Mat();
	}

	cv::Point2f center;
	center.x = image.size().width / 2;
	center.y = image.size().height / 2;
	std::cout << " center " << center << std::endl;
	std::cout << " angle " << angle << std::endl;
	std::cout << " flip " << flip << std::endl;
	cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);

	cv::Mat result;
	cv::warpAffine(image, result, rot_mat, cv::Size(image.cols, image.rows),
			CV_INTER_LINEAR);
	result.resize(image.cols, image.rows);
	result.resize(size, size);
	if (flip) {
		cv::flip(result, result, 0);
	}
	return result;
}

bool Mean_upper(const cv::Mat &image, const cv::Point2f &center) {
	int upper_points = 0;
	for (int ix = 0; ix < center.x; ++ix) {
		for (int iy = 0; iy < image.cols; ++iy) {
			upper_points += image.at<uint8_t>(ix, iy);
		}
	}
	int lower_points = 0;
	for (int ix = center.x; ix < image.rows; ++ix) {
		for (int iy = 0; iy < image.cols; ++iy) {
			lower_points += image.at<uint8_t>(ix, iy);
		}
	}
	if (upper_points < lower_points) {
		return false;
	}
	return true;
}

bool Compute_mean(const cv::Mat &image, cv::Point2f &center) {
	cv::Mat data;
	int data_column_index = 0;
	int center_col = 0;
	int center_row = 0;
	uint8_t value = 0;

	for (int row = 0; row < image.rows; ++row) {
		for (int col = 0; col < image.cols; ++col) {
			value = image.at<uint8_t>(row, col);
			center_col += col * value;
			center_row += row * value;
			data_column_index += value;
		}
	}
	if (data_column_index == 0) {
		return false;
	}
	center.x = float(center_row) / data_column_index;
	center.y = float(center_col) / data_column_index;
	return true;
}

uint8_t Compute_depth_value(float z_depth, float min_z, float max_z) {
	// z values have a range and the zero plane
	// thus the negative are for the max + something
	// [    +    |  - ]
	float z_tmp = 0;
	if (z_depth < 0) {
		z_tmp = max_z - z_depth;
	} else {
		z_tmp = z_depth - max_z;
	}
	float result =  (z_tmp*255 / (max_z - min_z));
	return  (uint8_t) result;
}

void Render_image(const cv::Mat &image_full,const cv::Mat &image_map,
		cv::Mat &image_result) {
	assert(image_full.rows == image_map.rows);
	assert(image_full.cols == image_map.cols);
	assert(image_full.cols == image_map.rows);
	assert(image_result.cols == image_map.rows);
	assert(image_result.rows == image_map.rows);

	const unsigned char *image_full_data;
	for (int ix = 0; ix < image_result.rows; ++ix) {
		for (int iy = 0; iy < image_result.cols; ++iy) {
			image_full_data = image_full.ptr<const unsigned char>(ix, iy);
			image_full_data += image_map.at<uint8_t>(ix, iy);
			image_result.at<uint8_t>(ix, iy) = *image_full_data;
		}
	}
}

void Render_image(const cv::Mat &image_full,const cv::Mat &image_map,
		cv::Mat &image_result, unsigned char channel) {
	assert(image_full.rows == image_map.rows);
	assert(image_full.cols == image_map.cols);
	assert(image_full.cols == image_map.rows);
	assert(image_result.cols == image_map.rows);
	assert(image_result.rows == image_map.rows);

	const unsigned char *image_full_data;
	for (int ix = 0; ix < image_result.rows; ++ix) {
		for (int iy = 0; iy < image_result.cols; ++iy) {
			image_full_data = image_full.ptr<const unsigned char>(ix, iy);
			if (image_map.at<uint8_t>(ix, iy) == channel) {
				image_full_data += channel;
				image_result.at<uint8_t>(ix, iy) = *image_full_data;
			}
		}
	}
}

void Render_image(const grasp_template::TemplateHeightmap &heightmap,
		double min_z, double max_z, cv::Mat &image_full, cv::Mat &image_map) {
	/*
	 unsigned int size = heightmap.getNumTilesX();
	 cv::Mat result = cv::Mat(size, size, CV_8UC4);
	 */
	Eigen::Vector3d eig_point;

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			heightmap.gridToWorldCoordinates(ix, iy, eig_point.x(),
					eig_point.y());

			double raw = heightmap.getGridTileRaw(ix, iy);
			if (heightmap.isUnset(raw) || heightmap.isEmpty(raw)) {
				eig_point.z() = 0;
			} else {
				eig_point.z() = heightmap.getGridTile(eig_point.x(),
						eig_point.y());
			}
			// the minus is just such that one has positive values
			// the actual value is pretty low since it is given in meters
			image_full.at<uint8_t>(ix, iy) = Compute_depth_value(
					static_cast<float>(eig_point.z()), min_z, max_z);

			if (heightmap.isSolid(raw)) {
				image_map.at<uint8_t>(ix, iy) = 0;
			} else if (heightmap.isFog(raw)) {
				image_map.at<uint8_t>(ix, iy) = 1;
			} else if (heightmap.isDontCare(raw)) {
				image_map.at<uint8_t>(ix, iy) = 2;
			} else if (heightmap.isTable(raw)) {
				image_map.at<uint8_t>(ix, iy) = 3;
			}
		}
	}
}

}
