/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         heightmap_difference.h

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#ifndef HEIGHTMAP_DIFFERENCE_H_
#define HEIGHTMAP_DIFFERENCE_H_

#include <vector>

#include <grasp_template/template_heightmap.h>
#include <grasp_template/Heightmap.h>

namespace grasp_template
{

class HeightmapDifference
{
public:

  HeightmapDifference(const Heightmap& t1, const Heightmap& t2);
  HeightmapDifference(const TemplateHeightmap& hm1, const TemplateHeightmap& hm2);

  std::vector<double> diff_;
  std::vector<std::pair<TileState, TileState> > states_;

private:

  void constr(const TemplateHeightmap& hm1, const TemplateHeightmap& hm2);
};

}  //namespace
#endif /* HEIGHTMAP_DIFFERENCE_H_ */
