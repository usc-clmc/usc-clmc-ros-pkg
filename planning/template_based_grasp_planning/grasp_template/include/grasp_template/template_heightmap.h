/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_heightmap.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef TEMPLATE_HEIGHTMAP_H_
#define TEMPLATE_HEIGHTMAP_H_

#include <limits>
#include <vector>

#include <visualization_msgs/Marker.h>
#include <grasp_template/Heightmap.h>

namespace grasp_template
{

enum TileState
{
  TS_SOLID, TS_FOG, TS_EMPTY, TS_DONTCARE, TS_TABLE, TS_UNSET
};

class TemplateHeightmap
{
public:

  TemplateHeightmap(unsigned int num_tiles_x, unsigned int num_tiles_y, double map_length_x, double map_length_y);
  TemplateHeightmap();

  /* constants */
  static const unsigned int TH_DEFAULT_NUM_TILES_X = 30;
  static const unsigned int TH_DEFAULT_NUM_TILES_Y = 30;
  static const double TH_UNSET_TILE, TH_EMPTY_TILE;
  static const double TH_DEPTH = 10; //means  -TH_DEPTH < values < TH_DEPTH
  static const double TH_FOG_ZERO = -100; //all fog values are shifted around TH_FOG_ZERO
  static const double TH_DONT_CARE_ZERO = -1000; //all dont_care values are shifted there
  static const double TH_TABLE_ZERO = -10000; //all table values are shifted there

  unsigned int getNumTilesX() const {return num_tiles_x_;};
  unsigned int getNumTilesY() const {return num_tiles_y_;};
  double getMapLengthX() const {return map_length_x_;};
  double getMapLengthY() const {return map_length_y_;};
  double getGridTileRaw(unsigned int ix, unsigned int iy) const;
  double getGridTileRaw(unsigned int i) const;
  double getGridTile(unsigned int i, TileState& ts) const;
  double getGridTile(unsigned int ix, unsigned int iy) const;
  double getGridTile(double x, double y) const;
  const std::vector<double>& getGrid() const {return grid_;};

  void setGridTileSolid(double x, double y, double value);
  void setGridTileEmpty(double x, double y);
  void setGridTileFog(double x, double y, double value);
  void setGridTileDontCare(double x, double y, double value);
  void setGridTileRaw(unsigned int ix, unsigned int iy, double value);
  void setGridTileTable(double x, double y, double value);
  void setGrid(const std::vector<double>& grid);

  bool isFog(double value) const;
  bool isDontCare(double value) const;
  bool isEmpty(double value) const;
  bool isSolid(double value) const;
  bool isUnset(double value) const;
  bool isTable(double value) const;

  void toHeightmapMsg(Heightmap& hm) const;
  std::vector<visualization_msgs::Marker> getVisualization(const std::string& ns, const std::string& frame_id, int id =
      0) const;
  void gridToWorldCoordinates(unsigned int ix, unsigned int iy, double& x, double& y) const;

private:

  unsigned int num_tiles_x_, num_tiles_y_;
  double map_length_x_, map_length_y_;
  std::vector<double> grid_;

  void constructClass(unsigned int num_tiles_x, unsigned int num_tiles_y, double map_length_x, double map_length_y);
  void transformToGridCoordinates(double x, double y, unsigned int& ix, unsigned int& iy) const;
};

} //namespace
#endif /* TEMPLATE_HEIGHTMAP_H_ */
