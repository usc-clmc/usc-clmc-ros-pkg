/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         data_grasp.cpp

 \author       Daniel Kappler
 \date         July 31, 2013

 *********************************************************************/

#include <deep_learning/data_grasp.h>

#include <deep_learning/def.h>


Data_grasp::Data_grasp(const geometry_msgs::Pose &pgripper_pose,const grasp_template::GraspTemplate &pgrasp_template,const std::string &puuid,float psuccess)
: gripper_pose(pgripper_pose),grasp_template(pgrasp_template),uuid(puuid),success(psuccess){

}
Data_grasp::Data_grasp(const geometry_msgs::Pose &pgripper_pose,const grasp_template::GraspTemplate &pgrasp_template)
: gripper_pose(pgripper_pose),grasp_template(pgrasp_template),uuid(UUID_NONE),success(SUCCESS_FALSE){

}
