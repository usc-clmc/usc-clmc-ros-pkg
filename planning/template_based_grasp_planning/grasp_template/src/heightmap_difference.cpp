/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         heightmap_difference.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <grasp_template/heightmap_difference.h>

namespace grasp_template
{

HeightmapDifference::HeightmapDifference(const TemplateHeightmap& hm1, const TemplateHeightmap& hm2)
{
  constr(hm1, hm2);
}

HeightmapDifference::HeightmapDifference(const Heightmap& t1, const Heightmap& t2)
{
  TemplateHeightmap hm1(t1.num_tiles_x, t1.num_tiles_y, t1.map_length_x, t1.map_length_y);
  hm1.setGrid(t1.heightmap);

  TemplateHeightmap hm2(t2.num_tiles_x, t2.num_tiles_y, t2.map_length_x, t2.map_length_y);
  hm2.setGrid(t2.heightmap);

  constr(hm1, hm2);
}

void HeightmapDifference::constr(const TemplateHeightmap& hm1, const TemplateHeightmap& hm2)
{
  assert(hm1.getGrid().size() == hm2.getGrid().size());

  const unsigned int n = hm1.getGrid().size();
  diff_.resize(n);
  states_.resize(n);
  for (unsigned int i = 0; i < n; i++)
  {
    double v1 = hm1.getGridTile(i, states_[i].first);
    double v2 = hm2.getGridTile(i, states_[i].second);

    if (states_[i].first == TS_EMPTY)
    {
      v1 = 0;
      if (states_[i].second == TS_DONTCARE)
      {
        v2 = 0;
      }
    }
    if (states_[i].second == TS_EMPTY)
    {
      v2 = 0;
      if (states_[i].first == TS_DONTCARE)
      {
        v1 = 0;
      }
    }
    diff_[i] = v1 - v2;
  }
}

} //namespace
