/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       work in progress: speedup template extraction

 \file          searchspace_grid.cpp

 \author        Alexander Herzog
 \date          April 1 2012

 *********************************************************************/

#include <vector>
#include <limits.h>
#include <cassert>
#include <cmath>

#include <geometry_msgs/Point32.h>
#include "grasp_template/searchspace_grid.h"

using namespace std;
using namespace Eigen;

namespace grasp_template
{

SearchspaceGrid::SearchspaceGrid(unsigned int num_y_tiles, unsigned int num_z_tiles)
{
	num_tiles_y_ = num_y_tiles;
	num_tiles_z_ = num_z_tiles;

	grid_.resize(num_tiles_y_ * num_tiles_z_);
}

void SearchspaceGrid::initialize(const Eigen::Matrix<double, 3, Eigen::Dynamic>
		pc)
{
	min_y_ = numeric_limits<double>::max();
	max_y_ = -numeric_limits<double>::max();
	min_z_ = numeric_limits<double>::max();
	max_z_ = -numeric_limits<double>::max();

	grid_.clear();

	for(unsigned int i = 0; i < pc.cols(); i++)
	{
		const Vector3d& p = pc.col(i);

		if(p.y() < min_y_) min_y_ = p.y();
		if(p.y() > max_y_) max_y_ = p.y();
		if(p.z() < min_z_) min_z_ = p.z();
		if(p.z() > max_z_) max_z_ = p.z();
	}

	/* compute grid structure */
	double y_epsylon = abs(max_y_ - min_y_) * 0.01;
	double z_epsylon = abs(max_z_ - min_z_) * 0.01;

	min_y_ -= y_epsylon;
	max_y_ += y_epsylon;
	min_z_ -= z_epsylon;
	max_z_ += z_epsylon;
	interval_y_ = (max_y_ - min_y_) / SG_GRANULARITY_Y;
	interval_z_ = (max_z_ - min_z_) / SG_GRANULARITY_Z;

	ROS_DEBUG_STREAM("grasp_teaching::SearchspaceGrid: min_y_ = " << min_y_
			<< ", max_y_ = " << max_y_
			<< ",  min_z_ = " << min_z_
			<< ", max_z_ = " << max_z_
			<< ", interval_y_ = " << interval_y_
			<< ", interval_z_ = " << interval_z_ );

	for(int i = 0; i < pc.cols(); i++)
	{
		/* apply grid to yz-plane */
		getTile(pc(1, i), pc(2, i)).push_back(i);
	}
};

std::vector<int>& SearchspaceGrid::getTile(double y, double z)
{
	if(y < min_y_ || y > max_y_ || z < min_z_ || z > max_z_)
	{
		ROS_ERROR("grasp_teaching::SearchspaceGrid::getTile cannot hold assert");
		assert(false);
	}

	unsigned int iy, iz;
	transformToGridCoordinates(y, z, iy, iz);

//	cout << " iy, iz: " << iy << " " << iz << endl;
	assert(iy < num_tiles_y_ && iz < num_tiles_z_);
	return getTile(iy, iz);
}


std::vector<int>& SearchspaceGrid::getTile(unsigned int iy, unsigned int iz)
{
	if(iy >= num_tiles_y_ || iz >= num_tiles_z_)
	{
		ROS_ERROR("SearchspaceGrid::getTile cannot hold assert");
		assert(false);
	}

//	cout << " iy, iz: " << iy << " " << iz << endl;
	return grid_[iz * num_tiles_y_ + iy];
}
const std::vector<int>& SearchspaceGrid::getTile (unsigned int iy,
		unsigned int iz) const
{
	if(iy >= num_tiles_y_ ||iz >= num_tiles_z_)
	{
		ROS_ERROR("SearchspaceGrid::getTile cannot hold assert");
		assert(false);
	}

	return grid_[iz * num_tiles_y_ + iy];
}

void SearchspaceGrid::transformToGridCoordinates(double y, double z, unsigned int& iy,
		unsigned int& iz) const
{
//	ROS_INFO("Entering trans with: x = %f, y = %f, map_length_x = %f, map_length_y = %f", x, y, map_length_x_, map_length_y_);
	if(y < min_y_ || y > max_y_ || z < min_z_ || z > max_z_)
	{
		ROS_ERROR("SearchspaceGrid::transformToGridCoordinates cannot hold assert");
		assert(false);
	}

	/* translate to grid origin */
	y -= min_y_;
	z -= min_z_;

	/* rasterize position */
	iy = static_cast<unsigned int>(floor(y / interval_y_) );
	iz = static_cast<unsigned int>(floor(z / interval_z_) );

	/* include upper border */
	if(y >= max_y_ - min_y_)
		iy = num_tiles_y_ - 1;
	if(z >= max_z_ - min_z_)
		iz = num_tiles_z_ - 1;
}

bool SearchspaceGrid::getIterator(const Eigen::Vector3d& point, const
		Eigen::Vector3d& normal, SsgIterator& it) const
{
//	assert(min_y_ <=  point.y() && point.y() < max_y_);
//	assert(min_z_ <=  point.z() && point.z() < max_z_);

	const double lamda_ymin = (min_y_ - point.y()) / normal.y();
	const double lamda_ymax = (max_y_ - point.y()) / normal.y();
	const double lamda_zmin = (min_z_ - point.z()) / normal.z();
	const double lamda_zmax = (max_z_ - point.z()) / normal.z();

//	if(abs(normal.y()) <= numeric_limits<double>::min())
//	{
//		lamda_s1 = lamda_zmin > lamda_zmax ? lamda_zmin : lamda_zmax;
//		lamda_s2 = lamda_zmin < lamda_zmax ? lamda_zmin : lamda_zmax;
//	}
//	else if(abs(normal.z()) <= numeric_limits<double>::min())
//	{
//		lamda_s1 = lamda_ymin > lamda_ymax ? lamda_ymin : lamda_ymax;
//		lamda_s2 = lamda_ymin < lamda_ymax ? lamda_ymin : lamda_ymax;
//	}
//	else
//	{
		vector<double> lamdas;
//		vector<bool> lamda_ysnap;
		const double ymin_z = lamda_ymin * normal.z() + point.z();
		if(min_z_ <= ymin_z && ymin_z < max_z_)
		{
			lamdas.push_back(lamda_ymin);
//			lamda_ysnap.push_back(true);
		}

		const double ymax_z = lamda_ymax * normal.z() + point.z();
		if(min_z_ < ymax_z && ymax_z <= max_z_)
		{
			lamdas.push_back(lamda_ymax);
//			lamda_ysnap.push_back(true);
		}

		const double zmin_y = lamda_zmin * normal.y() + point.y();
		if(min_y_ < zmin_y && zmin_y <= max_y_)
		{
			lamdas.push_back(lamda_zmin);
//			lamda_ysnap.push_back(false);
		}

		const double zmax_y = lamda_zmax * normal.y() + point.y();
		if(min_y_ <= zmax_y && zmax_y < max_y_)
		{
			lamdas.push_back(lamda_zmax);
//			lamda_ysnap.push_back(false);
		}

		if(lamdas.size() != 2)
			return false;

		double lamda_s1, lamda_s2;
		if(lamdas[0] < lamdas[1])
		{
			lamda_s1 = lamdas[0];
			lamda_s2 = lamdas[1];
//			it.ysnap_ = lamda_ysnap[1];
//			it.zsnap_ = !lamda_ysnap[1];
		}
		else
		{
			lamda_s1 = lamdas[1];
			lamda_s2 = lamdas[0];
//			it.ysnap_ = lamda_ysnap[0];
//			it.zsnap_ = !lamda_ysnap[0];
		}
//	}

//	it.border_epsilon_y_ = - abs(normal.y())/normal.y() * 0.5 * interval_y_;
//	it.border_epsilon_z_ = - abs(normal.z())/normal.z() * 0.5 * interval_z_;
	it.begin_y_ = point.y() + lamda_s1 * normal.y();
	it.begin_z_ = point.z() + lamda_s1 * normal.z();
	it.end_y_ = point.y() + lamda_s2 * normal.y();
	it.end_z_ = point.z() + lamda_s2 * normal.z();
	it.cur_y_ = it.end_y_;
	it.cur_z_ = it.end_z_;
//	it.step_y_ = -min(interval_y_, interval_z_) * normal.y() * 0.2;
//	it.step_z_ = -min(interval_y_, interval_z_) * normal.z() * 0.2;
	it.step_ = 1 / max(num_tiles_y_, num_tiles_z_) * 0.1;
	it.cur_lamda_ = 1;
//	it.dy_ = interval_y_;
//	it.dz_ = interval_z_;
	snap(it);

	return true;
}

void SearchspaceGrid::snap(SsgIterator& it) const
{
	/* translate to grid origin */
	double y = it.cur_y_ - min_y_;
	double z = it.cur_z_ - min_z_;

//	if(it.cur_y_ < 0) it.cur_y_ = 0;
//	if(it.cur_y_ > max_y_) it.cur_y_ = max_y_;
//	if(it.cur_z_ < 0) it.cur_z_ = 0;
//	if(it.cur_z_ > max_z_) it.cur_z_ = max_z_;

	/* rasterize position */
	it.cur_iy_ = static_cast<unsigned int>(floor(y / interval_y_) );
	it.cur_iz_ = static_cast<unsigned int>(floor(z / interval_z_) );

	if(it.cur_y_ < min_y_) it.cur_iy_ = 0;
	if(it.cur_z_ < min_z_) it.cur_iz_ = 0;
	if(it.cur_y_ >= max_y_) it.cur_iy_ = num_tiles_y_ - 1;
	if(it.cur_z_ >= max_z_) it.cur_iz_ = num_tiles_z_ - 1;
}

bool SearchspaceGrid::increase(SsgIterator& it)	//	check all exceptions
{
//	double next_y = it.cur_y_;
//	double next_z = it.cur_z_;
	double next_y, next_z;
	double next_lamda = 1;
	unsigned int next_iy, next_iz;

	do
	{
//		cout << "next_{y, z} = " << next_y << " " << next_z << endl;
//		next_y += it.step_y_;
//		next_z += it.step_z_;
		next_lamda -= it.step_;
		next_y = it.begin_y_ + (it.end_y_ - it.begin_y_) * next_lamda;
		next_z = it.begin_z_ + (it.end_z_ - it.begin_z_) * next_lamda;

		if(next_y < min_y_ || next_y > max_y_ || next_z < min_z_ || next_z > max_z_)
			return false;
		else
		{
			transformToGridCoordinates(next_y, next_z, next_iy, next_iz);
		}

	}while(next_iy == it.cur_iy_ && next_iz == it.cur_iz_);

	it.cur_iy_ = next_iy;
	it.cur_iz_ = next_iz;
	it.cur_y_ = next_y;
	it.cur_z_ = next_z;

	return true;
}


std::vector<int>& SearchspaceGrid::getTile(SsgIterator it)
{
	return getTile(it.cur_iy_, it.cur_iz_);
}

}	//namespace
