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

cv::Mat Render_image_4channel(
		const grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = heightmap.getNumTilesX();
	unsigned int offset = 0;
	/*
	 unsigned int size = (unsigned int) sqrt(
	 pow(heightmap.getNumTilesX(), 2) * 2);
	 unsigned int offset = (size - heightmap.getNumTilesX()) / 2;
	 */
	cv::Mat result = cv::Mat(size, size, CV_32FC4);

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
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
			float z = -static_cast<float>(eig_point.z());

			if (heightmap.isSolid(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[0] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[0] = 0;
			}
			if (heightmap.isFog(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[1] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[1] = 0;
			}

			if (heightmap.isDontCare(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[2] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[2] = 0;
			}

			if (heightmap.isTable(raw)) {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[3] = z;
			} else {
				result.at<cv::Vec4f>(offset + ix, offset + iy)[3] = 0;
			}
		}
	}
	return result;
}

cv::Mat Render_image_table(const grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = heightmap.getNumTilesX();
	unsigned int offset = 0;
	/*
	 unsigned int size = (unsigned int) sqrt(
	 pow(heightmap.getNumTilesX(), 2) * 2);
	 unsigned int offset = (size - heightmap.getNumTilesX()) / 2;
	 */
	cv::Mat result = cv::Mat(size, size, CV_32FC1);

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
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
			float z = -2000 * static_cast<float>(eig_point.z());

			if (heightmap.isTable(raw)) {
				result.at<float>(offset + ix, offset + iy) = z;
			} else {
				result.at<float>(offset + ix, offset + iy) = 0;
			}
		}
	}
	return result;
}

cv::Mat Render_image_solid(const grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = heightmap.getNumTilesX();
	unsigned int offset = 0;
	/*
	 unsigned int size = (unsigned int) sqrt(
	 pow(heightmap.getNumTilesX(), 2) * 2);
	 unsigned int offset = (size - heightmap.getNumTilesX()) / 2;
	 */
	cv::Mat result = cv::Mat(size, size, CV_32FC1);

	unsigned int counter = 0;

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
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
			float z = -2000 * static_cast<float>(eig_point.z());

			if (heightmap.isSolid(raw)) {
				result.at<float>(offset + ix, offset + iy) = z;
				if (z > 0) {
					counter += 1;
				}
			} else {
				result.at<float>(offset + ix, offset + iy) = 0;
			}
		}
	}
	if (counter > 10) {
		return result;
	}
	return cv::Mat();
}

cv::Mat Render_image_fog(const grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = heightmap.getNumTilesX();
	unsigned int offset = 0;
	cv::Mat result = cv::Mat(size, size, CV_32FC1);

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
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
			float z = -2000 * static_cast<float>(eig_point.z());

			if (heightmap.isFog(raw)) {
				result.at<float>(offset + ix, offset + iy) = z;
			} else {
				result.at<float>(offset + ix, offset + iy) = 0;
			}

		}
	}
	return result;
}

cv::Mat Render_image_dontcare(
		const grasp_template::TemplateHeightmap &heightmap) {
	// compute the diagonal and position the pixels in the center
	assert(heightmap.getNumTilesX() == heightmap.getNumTilesY());
	unsigned int size = heightmap.getNumTilesX();
	unsigned int offset = 0;
	/*
	 unsigned int size = (unsigned int) sqrt(
	 pow(heightmap.getNumTilesX(), 2) * 2);
	 unsigned int offset = (size - heightmap.getNumTilesX()) / 2;
	 */
	cv::Mat result = cv::Mat(size, size, CV_32FC1);

	for (unsigned int ix = 0; ix < heightmap.getNumTilesX(); ++ix) {
		for (unsigned int iy = 0; iy < heightmap.getNumTilesY(); ++iy) {

			Eigen::Vector3d eig_point;
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
			float z = -2000 * static_cast<float>(eig_point.z());

			if (heightmap.isDontCare(raw)) {
				result.at<float>(offset + ix, offset + iy) = z;
			} else {
				result.at<float>(offset + ix, offset + iy) = 0;
			}

		}
	}
	return result;
}

cv::Mat Normalize_image(const cv::Mat image,int size) {

	assert(image.rows == image.cols);
	if(sqrt(2*pow(size/2,2)) > (image.rows/2)) {
		std::cout << "size has to square has to fit inside the image for any rotation" << std::endl;
		return cv::Mat();
	}

	// the mean position should be on the center horizontal line
	cv::Point2f image_center;
	image_center.x = image.size().width / 2;
	image_center.y = image.size().height / 2;


	// we compute the mean of the image of all points which are not black
	// assuming that the data is uint8_t
	cv::Point2f center;
	if (!Compute_mean(image, center)) {
		return cv::Mat();
	}

	float direction_cols = center.x - image_center.x;
	float direction_rows = center.y - image_center.y;

	// the direction has to be normalized for the angle computation
	// angle is in negative horizont
	float direction_norm = direction_rows
			/ sqrt(
					direction_cols * direction_cols
							+ direction_rows * direction_rows);
	float angle = acos(-direction_norm) * 180.0f / (float) CV_PI;

	// if the it is below then the direction has to change
	if (center.x < image_center.x) {
		angle *= -1;
	}

	cv::Mat result = Rotate_image(image, image_center, -angle);

	// debugging
//	int block_size = 3;
//	for(int rows = -block_size; rows < block_size; ++rows){
//		for(int cols = -block_size; cols < block_size; ++cols){
//			result.at<uint8_t>(image_center.y+rows,image_center.x+cols) = 155;
//		}
//	}
	// debugging

	if (Mean_upper(result, image_center)) {
		cv::flip(result, result, 0);
	}
	result.resize(size,size);
	return result;
}

cv::Mat Rotate_image(const cv::Mat image, const cv::Point2f &center,
		double angle) {
	cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle, 1.0);
	cv::Mat result;
	//cv::warpAffine(image,result,rot_mat,cv::Size(image.cols,image.rows));
	cv::warpAffine(image, result, rot_mat, cv::Size(image.cols, image.rows),
			CV_INTER_LINEAR);
	result.resize(image.cols, image.rows);
	return result;
}

bool Mean_upper(const cv::Mat image, const cv::Point2f &center) {
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

bool Compute_mean(const cv::Mat image, cv::Point2f &center) {
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

}
