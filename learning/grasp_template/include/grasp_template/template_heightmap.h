/*
 * template_heightmap.h
 *
 *  Created on: Feb 11, 2011
 *      Author: herzog
 */

#ifndef TEMPLATE_HEIGHTMAP_H_
#define TEMPLATE_HEIGHTMAP_H_

#include <limits>

#include <Eigen3/Eigen>

namespace grasp_template
{

/* consts */
const unsigned int TH_DEFAULT_NUM_TILES_X = 10;
const unsigned int TH_DEFAULT_NUM_TILES_Y = 10;
const double TH_DEFAULT_MAP_LENGTH_X = 0.1;  // meter
const double TH_DEFAULT_MAP_LENGTH_Y = 0.1;  // meter

const double TH_EMPTY_TILE = std::numeric_limits<double>::min();

class TemplateHeightmap
{
public:

	TemplateHeightmap(unsigned int num_tiles_x = TH_DEFAULT_NUM_TILES_X,
			unsigned int num_tiles_y = TH_DEFAULT_NUM_TILES_Y,
			double map_length_x = TH_DEFAULT_MAP_LENGTH_X,
			double map_length_y = TH_DEFAULT_MAP_LENGTH_Y);

	unsigned int getNumTilesX() const{return num_tiles_x_;};
	unsigned int getNumTilesY() const{return num_tiles_y_;};
	double getMapLengthX() const{return map_length_x_;};
	double getMapLengthY() const{return map_length_y_;};


	/*
	 * access grid with array coordinates
	 */
	double getGridTile(unsigned int ix, unsigned int iy) const;
	void setGridTile(unsigned int ix, unsigned int iy, double value);

	/*
	 * access grid with world coordinates
	 */
	double getGridTile(double x, double y) const;
	void setGridTile(double x, double y, double value);

	/*
	 * interpolate empty tiles from surrounding filled tiles using bilinear
	 * interpolation
	 */
	void interpolateEmptyTiles();

	void gridToWorldCoordinates(unsigned int ix, unsigned int iy, double& x,
			double& y) const;
private:

	unsigned int num_tiles_x_, num_tiles_y_;
	double map_length_x_, map_length_y_;
	std::vector<double> grid_;

	void transformToGridCoordinates(double x, double y, unsigned int& ix,
			unsigned int& iy) const;

	inline bool minDistTile(const Eigen3::Vector2d& reference,
			unsigned int lower_bound, unsigned int upper_bound,
			unsigned int left_bound, unsigned int right_bound, Eigen3::Vector3d& result);

	/*
	 * maps 2D Point on Surface
	 * @param alpha if a * b = 0, alpha is the distance to the straight given
	 * by p + lambda * a
	 */
	inline double mapOnSurface(const Eigen3::Vector3d& p, const Eigen3::Vector3d& a,
			const Eigen3::Vector3d& b, const Eigen3::Vector2d& c, double& alpha);
};

}  //namespace

#endif /* TEMPLATE_HEIGHTMAP_H_ */
