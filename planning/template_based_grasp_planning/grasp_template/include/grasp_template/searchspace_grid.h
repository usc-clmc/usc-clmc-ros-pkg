/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       work in progress: speedup template extraction

 \file          searchspace_grid.h

 \author        Alexander Herzog
 \date          April 1 2012

 *********************************************************************/

#ifndef SEARCHSPACE_GRID_H_
#define SEARCHSPACE_GRID_H_

#include <vector>

#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud.h>

namespace grasp_template
{

struct SsgIterator
{
public:
//	double border_epsilon_y_, border_epsilon_z_;
	double begin_y_, begin_z_;
//	double step_y_, step_z_;
	double step_;
	double end_y_, end_z_;
	double cur_y_, cur_z_;
	double cur_lamda_;
	unsigned int cur_iy_, cur_iz_;
//	double dy_, dz_;
//	bool ysnap_, zsnap_;
};

const int  SG_GRANULARITY_Y = 20;
const int SG_GRANULARITY_Z = 20;

class SearchspaceGrid
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SearchspaceGrid(unsigned int num_y_tiles = SG_GRANULARITY_Y, unsigned int
			num_z_tiles = SG_GRANULARITY_Z);

	void initialize(const Eigen::Matrix<double, 3, Eigen::Dynamic> pc);

/*DEBUG, actually private */

	unsigned int num_tiles_y_, num_tiles_z_;
	double min_y_, max_y_, min_z_, max_z_, interval_y_, interval_z_;
	std::vector<std::vector<int> > grid_;
	std::vector<int>& getTile(double y, double z);
	std::vector<int>& getTile(SsgIterator it);
	std::vector<int>& getTile(unsigned int iy, unsigned int iz);
	const std::vector<int>& getTile (unsigned int iy, unsigned int iz) const;
	bool getIterator(const Eigen::Vector3d& p, const
			Eigen::Vector3d& n, SsgIterator& it) const;
	bool increase(SsgIterator& it);
	void transformToGridCoordinates(double y, double z, unsigned int& iy,
			unsigned int& iz) const;
private:



	void snap(SsgIterator& it) const;
};

}	//namespace
#endif /* SEARCHSPACE_GRID_H_ */
