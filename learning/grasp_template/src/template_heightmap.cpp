/*
 * template_heightmap.cpp
 *
 *  Created on: Feb 11, 2011
 *      Author: herzog
 */

#include <cassert>
#include <cmath>
#include <limits>

#include <ros/ros.h>

 #define EIGEN_USE_NEW_STDVECTOR
 #include <Eigen3/StdVector>
 #include <Eigen3/Eigen>

#include <grasp_template/template_heightmap.h>


using namespace std;

namespace grasp_template
{

TemplateHeightmap::TemplateHeightmap(unsigned int num_tiles_x, unsigned int num_tiles_y,
		double map_length_x, double map_length_y)
{
	num_tiles_x_ = num_tiles_x;
	num_tiles_y_ = num_tiles_y;
	map_length_x_ = map_length_x;
	map_length_y_ = map_length_y;

	grid_.clear();
	for(int i = 0; i < num_tiles_x_ * num_tiles_y_; i++)
	{
		grid_.push_back(TH_EMPTY_TILE);
	}
}

double TemplateHeightmap::getGridTile(unsigned int ix, unsigned int iy) const
{
	assert(/*0 <= ix &&*/ ix < num_tiles_x_);
	assert(/*0 <= iy &&*/ iy < num_tiles_y_);

	return grid_[iy * num_tiles_x_ + ix];
}

void TemplateHeightmap::setGridTile(unsigned int ix, unsigned int iy, double value)
{
	assert(/*0 <= ix &&*/ ix < num_tiles_x_);
	assert(/*0 <= iy &&*/ iy < num_tiles_y_);

	grid_[iy * num_tiles_x_ + ix] = value;
}

double TemplateHeightmap::getGridTile(double x, double y) const
{
	unsigned int ix, iy;
	transformToGridCoordinates(x ,y ,ix, iy);
	return getGridTile(ix, iy);
}

void TemplateHeightmap::setGridTile(double x, double y, double value)
{
//	ROS_INFO("Entering set with: x = %f, y = %f, map_length_x = %f, map_length_y = %f", x, y, map_length_x_, map_length_y_);
	unsigned int ix, iy;
	transformToGridCoordinates(x ,y ,ix, iy);
	setGridTile(ix, iy, value);

}

void TemplateHeightmap::transformToGridCoordinates(double x, double y, unsigned int& ix,
		unsigned int& iy) const
{
//	ROS_INFO("Entering trans with: x = %f, y = %f, map_length_x = %f, map_length_y = %f", x, y, map_length_x_, map_length_y_);
	const double tile_length_x = map_length_x_ / num_tiles_x_;
	const double tile_length_y = map_length_y_ / num_tiles_y_;

	/* translate to grid origin */
	x += map_length_x_ / 2.0;
	y += map_length_y_ / 2.0;

	/* DEBUG */
	if(!(0.0 <= x && x <= map_length_x_ && 0.0 <= y && y <= map_length_y_))
	{
		ROS_INFO("x = %f, y = %f, map_length_x = %f, map_length_y = %f", x, y, map_length_x_, map_length_y_);
	}
	/* DEBUG */
	assert(0.0 <= x && x <= map_length_x_);
	assert(0.0 <= y && y <= map_length_y_);

	/* rasterize position */
	ix = static_cast<unsigned int>(floor(x / tile_length_x) );
	iy = static_cast<unsigned int>(floor(y / tile_length_y) );

	/* as we use floor, be sure upper bounds are inside of grid */
	if(x == map_length_x_) ix--;
	if(y == map_length_y_) iy--;
}

 void TemplateHeightmap::interpolateEmptyTiles()
 {
 	for(unsigned int ix = 0; ix < num_tiles_x_; ++ix)
 	{
 		for(unsigned int iy = 0; iy < num_tiles_y_; ++iy)
 		{
 			double tile_value = getGridTile(ix, iy);
 			if(tile_value == TH_EMPTY_TILE)
 			{
 				vector<Eigen3::Vector3d, Eigen3::aligned_allocator<Eigen3::Vector3d> > reference_vectors;
 				Eigen3::Vector3d min_tile_container;
 				double lower_bound, upper_bound, left_bound, right_bound;

 				/* search first quadrant */
 				left_bound = ix;
 				lower_bound = iy + 1;
 				right_bound = num_tiles_x_ - 1;
 				upper_bound = num_tiles_y_ - 1;
 				if(minDistTile(*(new Eigen3::Vector2d(static_cast<double>(ix),
 						static_cast<double>(iy)) ), lower_bound, upper_bound,
 						left_bound, right_bound, min_tile_container) )
 				{
 					reference_vectors.push_back(min_tile_container);
 				}

 				/* search second quadrant */
 				left_bound = ix + 1;
 				lower_bound = 0;
 				right_bound = num_tiles_x_ - 1;
 				upper_bound = iy;
 				if(minDistTile(*(new Eigen3::Vector2d(static_cast<double>(ix),
 						static_cast<double>(iy)) ), lower_bound, upper_bound,
 						left_bound, right_bound, min_tile_container) )
 				{
 					reference_vectors.push_back(min_tile_container);
 				}

 				/* search third quadrant */
 				left_bound = 0;
 				lower_bound = 0;
 				right_bound = ix;
 				upper_bound = iy - 1;
 				if(minDistTile(*(new Eigen3::Vector2d(static_cast<double>(ix),
 						static_cast<double>(iy)) ), lower_bound, upper_bound,
 						left_bound, right_bound, min_tile_container) )
 				{
 					reference_vectors.push_back(min_tile_container);
 				}

 				/* search fourth quadrant */
 				left_bound = 0;
 				lower_bound = iy;
 				right_bound = ix - 1;
 				upper_bound = num_tiles_y_ - 1;
 				if(minDistTile(*(new Eigen3::Vector2d(static_cast<double>(ix),
 						static_cast<double>(iy)) ), lower_bound, upper_bound,
 						left_bound, right_bound, min_tile_container) )
 				{
 					reference_vectors.push_back(min_tile_container);
 				}

 				//TODO: CHECK IF 2D!!! VECTORS ARE LINEAR INDEPENDENT

 				assert(reference_vectors.size() != 0);

 				if(reference_vectors.size() == 1)
 				{
 					setGridTile(ix, iy, reference_vectors[0](2));
 					continue;

 				}else if(reference_vectors.size() == 2 || reference_vectors.size() == 3)
 				{
 					Eigen3::Vector2d v_c;
 					v_c(0) = static_cast<double>(ix);
 					v_c(1) = static_cast<double>(iy);

 					Eigen3::Vector3d v_p = reference_vectors[0];
 					Eigen3::Vector3d v_a = reference_vectors[1] - v_p;
 					Eigen3::Vector3d v_b;

 					if(reference_vectors.size() == 2)
 					{
 						v_b(0) = v_a(1);
 						v_b(1) = -v_a(0);
 						v_b(2) = 0;

 					}else if(reference_vectors.size() == 3)
 					{
 						v_b = reference_vectors[2] - v_p;
 					}

 					double dummy;
 					setGridTile(ix, iy, mapOnSurface(v_p, v_a, v_b, v_c, dummy));
 					continue;

 				}else
 				{
 					Eigen3::Vector2d v_c;
 					v_c(0) = static_cast<double>(ix);
 					v_c(1) = static_cast<double>(iy);
 					double dist_1, dist_2, dist_3, dist_4, val_1, val_2,
 							val_3, val_4;
 					Eigen3::Vector3d v_p, v_a, v_b;

 					//  first
 					v_p = reference_vectors[0];
 					v_a = reference_vectors[1] - v_p;

 					v_b(0) = v_a(1);
 					v_b(1) = -v_a(0);
 					v_b(2) = 0;

 					val_1 = mapOnSurface(v_p, v_a, v_b, v_c, dist_1);

 					//  second
 					v_p = reference_vectors[1];
 					v_a = reference_vectors[2] - v_p;

 					v_b(0) = v_a(1);
 					v_b(1) = -v_a(0);
 					v_b(2) = 0;

 					val_2 = mapOnSurface(v_p, v_a, v_b, v_c, dist_2);

 					//  third
 					v_p = reference_vectors[2];
 					v_a = reference_vectors[3] - v_p;

 					v_b(0) = v_a(1);
 					v_b(1) = -v_a(0);
 					v_b(2) = 0;

 					val_3 = mapOnSurface(v_p, v_a, v_b, v_c, dist_3);

 					//  fourth
 					v_p = reference_vectors[3];
 					v_a = reference_vectors[4] - v_p;

 					v_b(0) = v_a(1);
 					v_b(1) = -v_a(0);
 					v_b(2) = 0;

 					val_4 = mapOnSurface(v_p, v_a, v_b, v_c, dist_4);

 					const double result = val_1 * dist_1 + val_2 * dist_2 +
 							val_3 * dist_3 + val_4 * dist_4 /
 							(dist_1 + dist_2 + dist_3 + dist_4);
 					setGridTile(ix, iy, result);
 					continue;
 				}

 			}
 		}  // for iy
 	}  // for ix
 }

 inline bool TemplateHeightmap::minDistTile(const Eigen3::Vector2d& reference,
 		unsigned int lower_bound, unsigned int upper_bound,
 		unsigned int left_bound, unsigned int right_bound, Eigen3::Vector3d& result)
 {
 	if(left_bound > right_bound || lower_bound > upper_bound)
 		return false;

 	double min_dist_sqr = numeric_limits<double>::max();
 	bool min_exists = false;

 	for(unsigned int ix = left_bound; ix <= right_bound; ++ix)
 	{
 		for(unsigned int iy = lower_bound; iy <= upper_bound; ++iy)
 		{
 			const double tile = getGridTile(ix, iy);
 			if(tile != TH_EMPTY_TILE)
 			{
 				const double dx = static_cast<double>(ix) - reference(0);
 				const double dy = static_cast<double>(iy) - reference(1);

 				const double tile_dist_sqr = dx * dx + dy * dy;

 				if(tile_dist_sqr < min_dist_sqr)
 				{
 					result(0) = static_cast<double>(ix);
 					result(1) = static_cast<double>(iy);
 					result(2) = tile;

 					min_exists = true;
 				}
 			}
 		}
 	}

 	return min_exists;
 }

 inline double TemplateHeightmap::mapOnSurface(const Eigen3::Vector3d& p, const Eigen3::Vector3d& a,
 			const Eigen3::Vector3d& b, const Eigen3::Vector2d& c, double& alpha)
 {
 	Eigen3::Matrix2d m;
 	m(0, 0) = a(0);
 	m(0, 1) = b(0);
 	m(1, 0) = a(1);
 	m(1, 1) = b(1);

 	Eigen3::Vector2d t;
 	t(0) = c(0) - p(0);
 	t(1) = c(1) - p(1);

 	Eigen3::Vector2d s = m.inverse() * t;

 	alpha = b.norm() * abs(s(1));
 	return s(0) * a(2) + s(1) * b(2) + p(2);
 }

 void TemplateHeightmap::gridToWorldCoordinates(unsigned int ix, unsigned int iy, double& x,
 			double& y) const
 {
	 x = static_cast<double>(ix);
	 y = static_cast<double>(iy);

	 x -= map_length_x_ / 2.0;
	 y -= map_length_y_ / 2.0;

	 x += map_length_x_ / num_tiles_x_ / 2.0;
	 y += map_length_y_ / num_tiles_y_ / 2.0;
 }

}  //namespace
