/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_heightmap.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <cassert>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template/template_heightmap.h>

using namespace std;
using namespace Eigen;
using namespace visualization_msgs;

namespace grasp_template
{

const double TemplateHeightmap::TH_UNSET_TILE = numeric_limits<double>::max();
const double TemplateHeightmap::TH_EMPTY_TILE = numeric_limits<double>::max() / 2;

TemplateHeightmap::TemplateHeightmap(unsigned int num_tiles_x, unsigned int num_tiles_y, double map_length_x,
                                     double map_length_y)
{
  constructClass(num_tiles_x, num_tiles_y, map_length_x, map_length_y);
}

TemplateHeightmap::TemplateHeightmap()
{
  constructClass(TH_DEFAULT_NUM_TILES_X, TH_DEFAULT_NUM_TILES_Y, GraspTemplateParams::getTemplateWidth(),
                 GraspTemplateParams::getTemplateWidth());
}

void TemplateHeightmap::setGrid(const vector<double>& grid)
{
  assert(grid.size() == num_tiles_x_ * num_tiles_y_);
  grid_ = grid;
}

double TemplateHeightmap::getGridTileRaw(unsigned int ix, unsigned int iy) const
{
  assert(/*0 <= ix &&*/ix < num_tiles_x_);
  assert(/*0 <= iy &&*/iy < num_tiles_y_);
  return grid_[iy * num_tiles_x_ + ix];
}

double TemplateHeightmap::getGridTileRaw(unsigned int i) const
{
  assert(i < num_tiles_x_ * num_tiles_y_);
  return grid_[i];
}

void TemplateHeightmap::setGridTileRaw(unsigned int ix, unsigned int iy, double value)
{
  assert(/*0 <= ix &&*/ix < num_tiles_x_);
  assert(/*0 <= iy &&*/iy < num_tiles_y_);

  grid_[iy * num_tiles_x_ + ix] = value;
}

bool TemplateHeightmap::isFog(double value) const
{
  return value <= TH_FOG_ZERO + TH_DEPTH && value >= TH_FOG_ZERO - TH_DEPTH;
}

bool TemplateHeightmap::isDontCare(double value) const
{
  return value <= TH_DONT_CARE_ZERO + TH_DEPTH && value >= TH_DONT_CARE_ZERO - TH_DEPTH;
}

bool TemplateHeightmap::isEmpty(double value) const
{
  return value == TH_EMPTY_TILE;
}

bool TemplateHeightmap::isSolid(double value) const
{
  return value <= TH_DEPTH && value >= -TH_DEPTH;
}

bool TemplateHeightmap::isUnset(double value) const
{
  return value == TH_UNSET_TILE;
}

bool TemplateHeightmap::isTable(double value) const
{
  return value <= TH_TABLE_ZERO + TH_DEPTH && value >= TH_TABLE_ZERO - TH_DEPTH;
}

double TemplateHeightmap::getGridTile(double x, double y) const
{
  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);

  return getGridTile(ix, iy);
}

double TemplateHeightmap::getGridTile(unsigned int ix, unsigned int iy) const
{
  double raw_value = getGridTileRaw(ix, iy);

  if (isFog(raw_value))
    raw_value -= TH_FOG_ZERO;
  else if (isTable(raw_value))
    raw_value -= TH_TABLE_ZERO;
  else if (isDontCare(raw_value))
    raw_value -= TH_DONT_CARE_ZERO;

  return raw_value;
}

double TemplateHeightmap::getGridTile(unsigned int i, TileState& ts) const
{
  double raw_value = getGridTileRaw(i);

  if (isFog(raw_value))
  {
    raw_value -= TH_FOG_ZERO;
    ts = TS_FOG;
  }
  else if (isTable(raw_value))
  {
    raw_value -= TH_TABLE_ZERO;
    ts = TS_TABLE;
  }
  else if (isEmpty(raw_value))
  {
    ts = TS_EMPTY;
  }
  else if (isDontCare(raw_value))
  {
    raw_value -= TH_DONT_CARE_ZERO;
    ts = TS_DONTCARE;
  }
  else if (isUnset(raw_value))
  {
    ts = TS_UNSET;
  }
  else
  {
    ts = TS_SOLID;
  }

  return raw_value;
}

void TemplateHeightmap::setGridTileSolid(double x, double y, double value)
{
  /* handle out of range cases */
  if (value > TH_DEPTH)
  {
    value = TH_DEPTH;
  }
  else if (value < -TH_DEPTH)
  {
    value = TH_EMPTY_TILE;
  }

  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);
  setGridTileRaw(ix, iy, value);
}

void TemplateHeightmap::setGridTileEmpty(double x, double y)
{
  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);
  setGridTileRaw(ix, iy, TH_EMPTY_TILE);
}

void TemplateHeightmap::setGridTileFog(double x, double y, double value)
{
  /* handle out of range cases */
  if (value > TH_DEPTH)
  {
    value = TH_FOG_ZERO + TH_DEPTH;
  }
  else if (value < -TH_DEPTH)
  {
    value = TH_EMPTY_TILE;
  }
  else
  {
    value += TH_FOG_ZERO;
  }

  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);
  setGridTileRaw(ix, iy, value);
}

void TemplateHeightmap::setGridTileTable(double x, double y, double value)
{
  /* shift and/or cut off values to according intervals */
  if (value > TH_DEPTH)
  {
    value = TH_TABLE_ZERO + TH_DEPTH;
  }
  else if (value < -TH_DEPTH)
  {
    value = TH_EMPTY_TILE;
  }
  else
  {
    value += TH_TABLE_ZERO;
  }

  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);
  setGridTileRaw(ix, iy, value);
}

void TemplateHeightmap::setGridTileDontCare(double x, double y, double value)
{
  /* shift and/or cut off values to according intervals */
  if (value > TH_DEPTH)
  {
    value = TH_DONT_CARE_ZERO + TH_DEPTH;
  }
  else if (value < -TH_DEPTH)
  {
    value = TH_EMPTY_TILE;
  }
  else
  {
    value += TH_DONT_CARE_ZERO;
  }

  unsigned int ix, iy;
  transformToGridCoordinates(x, y, ix, iy);
  setGridTileRaw(ix, iy, value);
}

void TemplateHeightmap::toHeightmapMsg(Heightmap& hm) const
{
  hm.map_length_x = getMapLengthX();
  hm.map_length_y = getMapLengthY();
  hm.num_tiles_x = getNumTilesX();
  hm.num_tiles_y = getNumTilesY();
  hm.heightmap = getGrid();
}

vector<Marker> TemplateHeightmap::getVisualization(const string& ns, const string& frame_id, int id) const
{
  const double tile_length_x = getMapLengthX() / getNumTilesX();
  const double tile_length_y = getMapLengthY() / getNumTilesX();

  Marker measured_points;
  measured_points.header.frame_id = frame_id;
  measured_points.header.stamp = ros::Time::now();
  measured_points.ns = ns;
  measured_points.id = id;
  measured_points.type = Marker::POINTS;
  measured_points.action = Marker::ADD;
  measured_points.pose.orientation.w = 1.0;
  measured_points.scale.x = 0.4 * min(tile_length_x, tile_length_y);
  measured_points.scale.y = 0.4 * min(tile_length_x, tile_length_y);
  measured_points.color.r = 1.0;
  measured_points.color.a = 1.0;

  for (unsigned int ix = 0; ix < getNumTilesX(); ++ix)
  {
    for (unsigned int iy = 0; iy < getNumTilesY(); ++iy)
    {
      double p_x, p_y;
      gridToWorldCoordinates(ix, iy, p_x, p_y);
      geometry_msgs::Point p;
      p.x = static_cast<float> (p_x);
      p.y = static_cast<float> (p_y);
      p.z = static_cast<float> (getGridTile(ix, iy));

      if (getGridTile(ix, iy) == TH_EMPTY_TILE)
      {
        p.z = 0;
      }

      measured_points.points.push_back(p);
    }
  }

  vector<Marker> result;
  result.push_back(measured_points);
  return result;
}

void TemplateHeightmap::gridToWorldCoordinates(unsigned int ix, unsigned int iy, double& x, double& y) const
{
  x = static_cast<double> (ix) / num_tiles_x_ * map_length_x_;
  y = static_cast<double> (iy) / num_tiles_y_ * map_length_y_;

  x -= map_length_x_ / 2.0;
  y -= map_length_y_ / 2.0;

  x += map_length_x_ / num_tiles_x_ / 2.0;
  y += map_length_y_ / num_tiles_y_ / 2.0;
}

void TemplateHeightmap::transformToGridCoordinates(double x, double y, unsigned int& ix, unsigned int& iy) const
{
  const double tile_length_x = map_length_x_ / num_tiles_x_;
  const double tile_length_y = map_length_y_ / num_tiles_y_;

  /* translate to grid origin */
  x += map_length_x_ / 2.0;
  y += map_length_y_ / 2.0;

  /* For debugging */
  if (!(0.0 <= x && x <= map_length_x_ && 0.0 <= y && y <= map_length_y_))
  {
    ROS_DEBUG("grasp_teaching::TempalteHeightmap: x = %f, y = %f, map_length_x = %f, map_length_y = %f", x, y, map_length_x_, map_length_y_);
  }
  /* For debugging */
  assert(0.0 <= x && x <= map_length_x_);
  assert(0.0 <= y && y <= map_length_y_);

  /* rasterize position */
  ix = static_cast<unsigned int> (floor(x / tile_length_x));
  iy = static_cast<unsigned int> (floor(y / tile_length_y));

  /* as we use floor, make sure that upper bounds are inside of grid */
  if (x == map_length_x_)
  {
    ix--;
  }
  if (y == map_length_y_)
  {
    iy--;
  }
}

void TemplateHeightmap::constructClass(unsigned int num_tiles_x, unsigned int num_tiles_y, double map_length_x,
                                       double map_length_y)
{
  num_tiles_x_ = num_tiles_x;
  num_tiles_y_ = num_tiles_y;
  map_length_x_ = map_length_x;
  map_length_y_ = map_length_y;

  grid_.clear();
  for (unsigned int i = 0; i < num_tiles_x_ * num_tiles_y_; i++)
  {
    grid_.push_back(TH_UNSET_TILE);
  }
}

} //namespace
