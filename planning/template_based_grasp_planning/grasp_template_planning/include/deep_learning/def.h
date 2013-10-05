/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         log_loader.h

 \author       Daniel Kappler
 \date         July 30, 2013

 *********************************************************************/

#ifndef DEF_H
#define DEF_H

#include <Eigen/Eigen>

namespace deep_learning {
const std::string UUID_NONE = "__NONE__";
const int SUCCESS_FALSE = 0;
const int SUCCESS_TRUE = 1;
const std::string DATABASE_GRASP_TOPIC = "deep_learning_database_grasp";
// arm
//const Eigen::Vector3d BOUNDING_BOX_CORNER_1(-0.12, -0.12, -0.07);
//const Eigen::Vector3d BOUNDING_BOX_CORNER_2(0.9, 0.12, 0.15);
// arm
// pr2
//const Eigen::Vector3d BOUNDING_BOX_CORNER_1(-0.005, -0.12, -0.07);
//const Eigen::Vector3d BOUNDING_BOX_CORNER_2(0.16, 0.12, 0.02);
// pr2
const Eigen::Vector3d BOUNDING_BOX_CORNER_1(-0.12, -0.12, -0.12);
const Eigen::Vector3d BOUNDING_BOX_CORNER_2(0.12, 0.12, 0.12);

const std::string DATA_GRASP_PATH_BAGFILE = "FROM_DATA_GRASP";
}

#endif /*DEF_H*/
