/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <stomp_motion_planner/stomp_collision_space.h>
#include <planning_environment/util/construct_object.h>
#include <sstream>

namespace stomp_motion_planner
{

StompCollisionSpace::StompCollisionSpace():
  node_handle_("~"),distance_field_(NULL),monitor_(NULL)
  //,collision_map_subscriber_(root_handle_,"collision_map_occ",1)
{
}

StompCollisionSpace::~StompCollisionSpace()
{
  if(distance_field_) {
    delete distance_field_;
  }
  //delete collision_map_filter_;
  //delete collision_object_filter_;
  //delete collision_object_subscriber_;
}

bool StompCollisionSpace::init(planning_environment::CollisionSpaceMonitor* monitor, double max_radius_clearance, std::string& reference_frame)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  double resolution;

  reference_frame_ = reference_frame;

  node_handle_.param("collision_space/size_x", size_x, 2.0);
  node_handle_.param("collision_space/size_y", size_y, 3.0);
  node_handle_.param("collision_space/size_z", size_z, 4.0);
  node_handle_.param("collision_space/origin_x", origin_x, 0.1);
  node_handle_.param("collision_space/origin_y", origin_y, -1.5);
  node_handle_.param("collision_space/origin_z", origin_z, -2.0);
  node_handle_.param("collision_space/resolution", resolution, 0.02);
  node_handle_.param("collision_space/field_bias_x", field_bias_x_, 0.0);
  node_handle_.param("collision_space/field_bias_y", field_bias_y_, 0.0);
  node_handle_.param("collision_space/field_bias_z", field_bias_z_, 0.0);
  resolution_ = resolution;
  max_expansion_ = max_radius_clearance;

  //initCollisionCuboids();

  distance_field_ = new distance_field::PropagationDistanceField(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, max_radius_clearance);

  monitor_ = monitor;
  //now setting up robot bodies for potential inclusion in the distance field
  loadRobotBodies();

  XmlRpc::XmlRpcValue coll_ops;

  if(!node_handle_.hasParam("stomp_collision_operations")) {
    ROS_WARN("No default collision operations specified");
  } else {

    node_handle_.getParam("stomp_collision_operations", coll_ops);
    
    if(coll_ops.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_WARN("stomp_collision_operations is not an array");
    } else {
      
      if(coll_ops.size() == 0) {
        ROS_WARN("No collision operations in stomp collision operations");
      } else {
        
        for(int i = 0; i < coll_ops.size(); i++) {
          if(!coll_ops[i].hasMember("object1") || !coll_ops[i].hasMember("object2") || !coll_ops[i].hasMember("operation")) {
            ROS_WARN("All collision operations must have two objects and an operation");
            continue;
          }
          std::string object1 = std::string(coll_ops[i]["object1"]);
          std::string object2 = std::string(coll_ops[i]["object2"]);
          std::string operation = std::string(coll_ops[i]["operation"]);
          if(operation == "enable") {
            if(planning_group_link_names_.find(object1) == planning_group_link_names_.end()) {
              ROS_WARN_STREAM("Object 1 must be a recognized planning group and " << object1 << " is not");
              continue;
            }
            if(distance_include_links_.find(object1) == distance_include_links_.end()) {
              std::vector<std::string> emp;
              distance_include_links_[object1] = emp;
            }
            std::vector<std::string>& include_links = distance_include_links_[object1];
            if(planning_group_link_names_.find(object2) == planning_group_link_names_.end()) {
              include_links.push_back(object2);
              ROS_DEBUG_STREAM("Link " << object1 << " adding include for link " << object2);
            } else {
              ROS_DEBUG_STREAM("Link " << object1 << " adding include for group " << object2 << " size " << planning_group_link_names_.find(object2)->second.size() );
              include_links.insert(include_links.end(), planning_group_link_names_.find(object2)->second.begin(),
                                   planning_group_link_names_.find(object2)->second.end());
            }
          } else if(operation == "disable") {
            if(planning_group_link_names_.find(object1) == planning_group_link_names_.end()) {
              ROS_WARN_STREAM("Object 1 must be a recognized planning group and " << object1 << " is not");
              continue;
            }
            if(distance_exclude_links_.find(object1) == distance_exclude_links_.end()) {
              std::vector<std::string> emp;
              distance_exclude_links_[object1] = emp;
            }
            std::vector<std::string>& exclude_links = distance_exclude_links_[object1];
            if(planning_group_link_names_.find(object2) == planning_group_link_names_.end()) {
              exclude_links.push_back(object2);
              ROS_DEBUG_STREAM("Link " << object1 << " adding exclude for link " << object2);
            } else {
              ROS_DEBUG_STREAM("Link " << object1 << " adding exclude for group " << object2 << " size " << planning_group_link_names_.find(object2)->second.size() );
              exclude_links.insert(exclude_links.end(), planning_group_link_names_.find(object2)->second.begin(),
                                   planning_group_link_names_.find(object2)->second.end());
            }
          } else {
            ROS_WARN_STREAM("Unrecognized collision operation " << operation << ". Must be enable or disable");
            continue;
          }
        }
      }
    }
  }

 //  collision_map_filter_ = new tf::MessageFilter<mapping_msgs::CollisionMap>(collision_map_subscriber_,tf_,reference_frame_,1);
//   collision_map_filter_->registerCallback(boost::bind(&StompCollisionSpace::collisionMapCallback, this, _1));

//   collision_object_subscriber_ = new message_filters::Subscriber<mapping_msgs::CollisionObject>(root_handle_, "collision_object", 1024);
//   collision_object_filter_ = new tf::MessageFilter<mapping_msgs::CollisionObject>(*collision_object_subscriber_, tf_, reference_frame_, 1024);
//   collision_object_filter_->registerCallback(boost::bind(&StompCollisionSpace::collisionObjectCallback, this, _1));


  ROS_INFO("Initialized stomp collision space in %s reference frame with %f expansion radius.", reference_frame_.c_str(), max_expansion_);
  return true;
}

void StompCollisionSpace::setStartState(const StompRobotModel::StompPlanningGroup& planning_group, const motion_planning_msgs::RobotState& robot_state) {

  ros::WallTime start = ros::WallTime::now();

  monitor_->waitForState();

  planning_models::KinematicState state(monitor_->getKinematicModel());

  distance_field_->reset();

  std::vector<btVector3> all_points;

  monitor_->getEnvironmentModel()->lock();
  monitor_->setRobotStateAndComputeTransforms(robot_state, state);

  //planning_models::KinematicState state(planning_models::KinematicState(*(monitor_->getRobotState())));
  monitor_->setCollisionSpace();

  std::string root_name = monitor_->getKinematicModel()->getRoot()->getName();

  //now we need to back out this transform
  const btTransform& cur = state.getJointState(root_name)->getVariableTransform();
  btTransform cur_copy(cur);

  //now get the body in the identity transform
  btTransform id;
  id.setIdentity();
  state.getJointState(root_name)->setJointStateValues(id);
  state.updateKinematicLinks();

  updateRobotBodiesPoses(state);

  addAllBodiesButExcludeLinksToPoints(planning_group.name_, all_points);
  addCollisionObjectsToPoints(all_points, cur_copy);

  ROS_INFO_STREAM("All points size " << all_points.size());
  
  distance_field_->addPointsToField(all_points);
  distance_field_->visualize(0*max_expansion_, 0.01*max_expansion_, monitor_->getWorldFrameId(), cur_copy, ros::Time::now());

  monitor_->getEnvironmentModel()->unlock();  

  ros::WallDuration t_diff = ros::WallTime::now() - start;
  ROS_INFO_STREAM("Took " << t_diff.toSec() << " to set distance field");
}

void StompCollisionSpace::addCollisionObjectsToPoints(std::vector<btVector3>& points, const btTransform& cur_transform)
{
  btTransform inv = cur_transform.inverse();
  const collision_space::EnvironmentObjects *eo = monitor_->getEnvironmentModel()->getObjects();
  std::vector<std::string> ns = eo->getNamespaces();
  for (unsigned int i = 0 ; i < ns.size() ; ++i)
  {
    const collision_space::EnvironmentObjects::NamespaceObjects &no = eo->getObjects(ns[i]);
    const unsigned int n = no.shape.size();
    
    //special case for collision map points
    if(ns[i] == "points") {
      //points.reserve(points.size()+n);
      for(unsigned int j = 0; j < n;  ++j) 
      {
        points.push_back(inv*no.shapePose[j].getOrigin());
      }
      continue;
    }
    for(unsigned int j = 0; j < n; j++) {
      if (no.shape[j]->type == shapes::MESH) {
        bodies::Body *body = bodies::createBodyFromShape(no.shape[j]);
        body->setPose(inv*no.shapePose[j]);
        std::vector<btVector3> body_points;
        getVoxelsInBody(*body, body_points);
        points.insert(points.end(), body_points.begin(), body_points.end());
        delete body;
      } else {
        geometric_shapes_msgs::Shape object;
        if(!planning_environment::constructObjectMsg(no.shape[j], object)) {
          ROS_WARN("Shap cannot be converted");
          continue;
        }
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(no.shapePose[j], pose);
        KDL::Rotation rotation = KDL::Rotation::Quaternion(pose.orientation.x,
                                                           pose.orientation.y,
                                                           pose.orientation.z,
                                                           pose.orientation.w);
        KDL::Vector position(pose.position.x, pose.position.y, pose.position.z);
        KDL::Frame f(rotation, position);
        if (object.type == geometric_shapes_msgs::Shape::CYLINDER)
        {
          if (object.dimensions.size() != 2) {
            ROS_INFO_STREAM("Cylinder must have exactly 2 dimensions, not " 
                            << object.dimensions.size()); 
            continue;
          }
          // generate points:
          double radius = object.dimensions[0];
          //ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);
          
          double xlow = pose.position.x - object.dimensions[0];
          double ylow = pose.position.y - object.dimensions[0];
          double zlow = pose.position.z - object.dimensions[1]/2.0;

          //ROS_INFO("pose " << pose.position.x << " " << pose.position.y);
          
          for(double x = xlow; x <= xlow+object.dimensions[0]*2.0+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[0]*2.0+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[1]+resolution_; z += resolution_) {
                double xdist = fabs(pose.position.x-x);
                double ydist = fabs(pose.position.y-y);
                //if(pose.position.z-z == 0.0) {
                //  ROS_INFO_STREAM("X " << x << " Y " << y << " Dists " << xdist << " " << ydist << " Rad " << sqrt(xdist*xdist+ydist*ydist));
                // }
                if(sqrt(xdist*xdist+ydist*ydist) <= radius) {
                  KDL::Vector p(pose.position.x-x,pose.position.y-y,pose.position.z-z);                  
                  KDL::Vector p2;
                  p2 = f*p;
                  points.push_back(inv*btVector3(p2(0),p2(1),p2(2)));
                }
              }
            }
          }
        } else if (object.type == geometric_shapes_msgs::Shape::BOX) {
          if(object.dimensions.size() != 3) {
            ROS_INFO_STREAM("Box must have exactly 3 dimensions, not " 
                            << object.dimensions.size());
            continue;
          }
              
          double xlow = pose.position.x - object.dimensions[0]/2.0;
          double ylow = pose.position.y - object.dimensions[1]/2.0;
          double zlow = pose.position.z - object.dimensions[2]/2.0;
          
          for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
            for(double y = ylow; y <= ylow+object.dimensions[1]+resolution_; y += resolution_) {
              for(double z = zlow; z <= zlow+object.dimensions[2]+resolution_; z += resolution_) {
                KDL::Vector p(pose.position.x-x,pose.position.y-y,pose.position.z-z);                  
                KDL::Vector p2;
                p2 = f*p;
                points.push_back(inv*btVector3(p2(0),p2(1),p2(2)));
              }
            }
          }
        } 
      }
    }
  }
}

// void StompCollisionSpace::collisionMapCallback(const mapping_msgs::CollisionMapConstPtr& collision_map)
// {
//   return;

//   // @TODO transform the collision map to the required frame!!
//   if (mutex_.try_lock())
//   {
//     ros::WallTime start = ros::WallTime::now();
//     distance_field_->reset();
//     ROS_INFO("Reset prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());
//     start = ros::WallTime::now();
//     distance_field_->addPointsToField(cuboid_points_);
//     distance_field_->addCollisionMapToField(*collision_map);
//     mutex_.unlock();
//     ROS_INFO("Updated prop distance_field in %f sec", (ros::WallTime::now() - start).toSec());

//     distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, collision_map->header.frame_id, collision_map->header.stamp);

//   }
//   else
//   {
//     ROS_INFO("Skipped collision map update because planning is in progress.");
//   }
// }

// void StompCollisionSpace::collisionObjectCallback(const mapping_msgs::CollisionObjectConstPtr &collisionObject) {
//   if (collisionObject->operation.operation == mapping_msgs::CollisionObjectOperation::ADD)
//   {
//     if (mutex_.try_lock())
//     {
//       std::vector<btVector3> points;
//       for (size_t i=0; i<collisionObject->shapes.size(); ++i)
//       {
//         const geometric_shapes_msgs::Shape& object = collisionObject->shapes[i];
//         const geometry_msgs::Pose& pose = collisionObject->poses[i];
//         if (object.type == geometric_shapes_msgs::Shape::CYLINDER)
//         {
//           ROS_INFO("Got cylinder");

//           if (object.dimensions.size() != 2) {
//             ROS_INFO_STREAM("Cylinder must have exactly 2 dimensions, not " 
//                             << object.dimensions.size()); 
//             continue;
//           }
//           KDL::Rotation rotation = KDL::Rotation::Quaternion(pose.orientation.x,
//                                                              pose.orientation.y,
//                                                              pose.orientation.z,
//                                                              pose.orientation.w);
//           KDL::Vector position(pose.position.x, pose.position.y, pose.position.z);
//           KDL::Frame f(rotation, position);
//           // generate points:
//           double radius = object.dimensions[0];
//           double length = object.dimensions[1];
//           KDL::Vector p(0,0,0);
//           KDL::Vector p2;
//           double xdiv = object.dimensions[0]/resolution_;
//           double ydiv = object.dimensions[0]/resolution_;
//           double zdiv = object.dimensions[1]/resolution_;

//           //ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);

//           double xlow = pose.position.x - object.dimensions[0]/2.0;
//           double ylow = pose.position.y - object.dimensions[0]/2.0;
//           double zlow = pose.position.z - object.dimensions[1]/2.0;

//           for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
//             for(double y = ylow; y <= ylow+object.dimensions[0]+resolution_; y += resolution_) {
//               for(double z = zlow; z <= zlow+object.dimensions[1]+resolution_; z += resolution_) {
//                 double xdist = fabs(pose.position.x-x);
//                 double ydist = fabs(pose.position.y-y);
//                 if(sqrt(xdist*xdist+ydist*ydist) < radius) {
//                   points.push_back(btVector3(x,y,z));
//                 }
//               }
//             }
//           }
//           //double spacing = resolution_;//radius/2.0;
//           //int num_points = ceil(length/spacing)+1;
//           //spacing = length/(num_points-1.0);
//           //for (int i=0; i<num_points; ++i)
//           //{
//           //  p(2) = -length/2.0 + i*spacing;
//           //  p2 = f*p;
//           //  points.push_back(btVector3(p2(0), p2(1), p2(2)));
//           //}
//         } else if (object.type == geometric_shapes_msgs::Shape::BOX) {
//           if(object.dimensions.size() != 3) {
//             ROS_INFO_STREAM("Box must have exactly 3 dimensions, not " 
//                             << object.dimensions.size());
//             continue;
//           }
          
//           double xdiv = object.dimensions[0]/resolution_;
//           double ydiv = object.dimensions[1]/resolution_;
//           double zdiv = object.dimensions[2]/resolution_;

//           ROS_INFO_STREAM("Divs " << xdiv << " " << ydiv << " " << zdiv);

//           double xlow = pose.position.x - object.dimensions[0]/2.0;
//           double ylow = pose.position.y - object.dimensions[1]/2.0;
//           double zlow = pose.position.z - object.dimensions[2]/2.0;

//           for(double x = xlow; x <= xlow+object.dimensions[0]+resolution_; x += resolution_) {
//             for(double y = ylow; y <= ylow+object.dimensions[1]+resolution_; y += resolution_) {
//               for(double z = zlow; z <= zlow+object.dimensions[2]+resolution_; z += resolution_) {
//                 points.push_back(btVector3(x,y,z));
//               }
//             }
//           }
//         } else if (object.type == geometric_shapes_msgs::Shape::MESH) {
//           // //adding the vertices themselves
//           // for(std::vector<geometry_msgs::Point>::const_iterator it = object.vertices.begin();
//           //     it != object.vertices.end();
//           //     it++) {
//           //   points.push_back(btVector3(it->x, it->y, it->z));
//           // }
//           // //now interpolating for big triangles
//           // for (size_t i=0; i<object.triangles.size(); i+=3)
//           // {
//           //   btVector3 v0( object.vertices.at( object.triangles.at(i+0) ).x,
//           //                 object.vertices.at( object.triangles.at(i+0) ).y,
//           //                 object.vertices.at( object.triangles.at(i+0) ).z);
//           //   btVector3 v1( object.vertices.at( object.triangles.at(i+1) ).x,
//           //                 object.vertices.at( object.triangles.at(i+1) ).y,
//           //                 object.vertices.at( object.triangles.at(i+1) ).z);
//           //   btVector3 v2( object.vertices.at( object.triangles.at(i+2) ).x,
//           //                 object.vertices.at( object.triangles.at(i+2) ).y,
//           //                 object.vertices.at( object.triangles.at(i+2) ).z);
//           //   std::vector<btVector3> triangleVectors = interpolateTriangle(v0, v1, v2, resolution_);
//           //   points.insert(points.end(), triangleVectors.begin(), triangleVectors.end());
//           // }
//         } else {
//           ROS_WARN("Attaching objects of non-cylinder types is not supported yet!");
//         }
//       }
//       ROS_INFO_STREAM("Trying to add " << points.size() << " to distance field");  
//       distance_field_->reset();
//       distance_field_->addPointsToField(cuboid_points_);
//       distance_field_->addPointsToField(points);
//       mutex_.unlock();
      
//       distance_field_->visualize(0.895*max_expansion_, 0.9*max_expansion_, collisionObject->header.frame_id, collisionObject->header.stamp);
//     }
//   }
// }

static std::string intToString(int i)
{
  std::ostringstream oss;
  oss << i;
  return oss.str();
}

void StompCollisionSpace::initCollisionCuboids()
{
  int index=1;
  while (node_handle_.hasParam(std::string("collision_space/cuboids/cuboid")+intToString(index)+"/size_x"))
  {
    addCollisionCuboid(std::string("collision_space/cuboids/cuboid")+intToString(index));
    index++;
  }

}

void StompCollisionSpace::addCollisionCuboid(const std::string param_name)
{
  double size_x, size_y, size_z;
  double origin_x, origin_y, origin_z;
  if (!node_handle_.getParam(param_name+"/size_x", size_x))
    return;
  if (!node_handle_.getParam(param_name+"/size_y", size_y))
    return;
  if (!node_handle_.getParam(param_name+"/size_z", size_z))
    return;
  if (!node_handle_.getParam(param_name+"/origin_x", origin_x))
    return;
  if (!node_handle_.getParam(param_name+"/origin_y", origin_y))
    return;
  if (!node_handle_.getParam(param_name+"/origin_z", origin_z))
    return;

  if (size_x<0 || size_y<0 || size_z<0)
    return;

  // add points:
  int num_points=0;
  for (double x=origin_x; x<=origin_x+size_x; x+=resolution_)
    for (double y=origin_y; y<=origin_y+size_y; y+=resolution_)
      for (double z=origin_z; z<=origin_z+size_z; z+=resolution_)
      {
        cuboid_points_.push_back(btVector3(x,y,z));
        ++num_points;
      }
  ROS_INFO("Added %d points for collision cuboid %s", num_points, param_name.c_str());
}

void StompCollisionSpace::loadRobotBodies() {
  planning_group_link_names_.clear();

  planning_group_link_names_ = monitor_->getCollisionModels()->getPlanningGroupLinks();

  ROS_DEBUG_STREAM("Planning group links size " << planning_group_link_names_.size());

  for(std::map<std::string, std::vector<std::string> >::iterator it1 = planning_group_link_names_.begin();
      it1 != planning_group_link_names_.end();
      it1++)
  {
    ROS_DEBUG_STREAM("Stomp loading group " << it1->first);

    for(std::vector<std::string>::iterator it2 = it1->second.begin();
        it2 != it1->second.end();
        it2++) {
      const planning_models::KinematicModel::LinkModel* link = monitor_->getCollisionModels()->getKinematicModel()->getLinkModel(*it2);
      if(link != NULL) {
        planning_group_bodies_[it1->first].push_back(bodies::createBodyFromShape(link->getLinkShape()));
      } else {
        ROS_WARN_STREAM("Error - no link for name " << *it2);
      }
      //planning_group_bodies_[it1->first].back()->setPadding(padding_);
    }
  }
}

void StompCollisionSpace::updateRobotBodiesPoses(const planning_models::KinematicState& state)
{
  for(std::map<std::string, std::vector<bodies::Body *> >::iterator it1 = planning_group_bodies_.begin();
      it1 != planning_group_bodies_.end();
      it1++) {
    std::vector<std::string>& v = planning_group_link_names_[it1->first];
    for(unsigned int i = 0; i < it1->second.size(); i++) {
      (it1->second)[i]->setPose(state.getLinkState(v[i])->getGlobalLinkTransform());
    }
  }
}

void StompCollisionSpace::addBodiesInGroupToPoints(const std::string& group, std::vector<btVector3>& body_points) {
  
  if(group == std::string("all")) {
    for(std::map<std::string, std::vector<bodies::Body *> >::iterator it1 = planning_group_bodies_.begin();
        it1 != planning_group_bodies_.end();
        it1++) {
      for(unsigned int i = 0; i < it1->second.size(); i++) {
        std::vector<btVector3> single_body_points;
        getVoxelsInBody((*it1->second[i]), single_body_points);
        ROS_DEBUG_STREAM("Group " << it1->first << " link num " << i << " points " << single_body_points.size());
        body_points.insert(body_points.end(), single_body_points.begin(), single_body_points.end());
      }
    }
  } else {
    if(planning_group_bodies_.find(group) != planning_group_bodies_.end()) {
      std::vector<bodies::Body *>& bodies = planning_group_bodies_[group];
      for(unsigned int i = 0; i < bodies.size(); i++) {
        std::vector<btVector3> single_body_points;
        getVoxelsInBody(*(bodies[i]), single_body_points);
        ROS_DEBUG_STREAM("Group " << group << " link num " << i << " points " << single_body_points.size());
        body_points.insert(body_points.end(), single_body_points.begin(), single_body_points.end());
      }
    } else {
      ROS_WARN_STREAM("Group " << group << " not found in planning groups");
    }
  }
}

void StompCollisionSpace::addAllBodiesButExcludeLinksToPoints(std::string group_name, std::vector<btVector3>& body_points) {

  std::vector<std::string> exclude_links;
  if(distance_exclude_links_.find(group_name) != distance_exclude_links_.end()) 
  {
    exclude_links = distance_exclude_links_[group_name];
  }

  //now go through include and see if we have to add anything back
  if(distance_include_links_.find(group_name) != distance_include_links_.end()) {
    for(std::vector<std::string>::iterator it = distance_include_links_[group_name].begin();
        it != distance_include_links_[group_name].end();
        it++) {
      std::vector<std::string>::iterator f = find(exclude_links.begin(), exclude_links.end(), *it);
      if(f != exclude_links.end()) {
        exclude_links.erase(f);
      }
    }
  }

  for(std::map<std::string, std::vector<bodies::Body *> >::iterator it1 = planning_group_bodies_.begin();
      it1 != planning_group_bodies_.end();
      it1++) {
    std::vector<std::string>& group_link_names = planning_group_link_names_[it1->first];

    for(unsigned int i = 0; i < it1->second.size(); i++) {
      if(find(exclude_links.begin(), exclude_links.end(),group_link_names[i]) == exclude_links.end()) {
        std::vector<btVector3> single_body_points;
        getVoxelsInBody((*it1->second[i]), single_body_points);
        ROS_DEBUG_STREAM("Group " << it1->first << " link " << group_link_names[i] << " points " << single_body_points.size());
        body_points.insert(body_points.end(), single_body_points.begin(), single_body_points.end());
      }
    }
  }
}

void StompCollisionSpace::getVoxelsInBody(const bodies::Body &body, std::vector<btVector3> &voxels)
{
  bodies::BoundingSphere bounding_sphere;

  body.computeBoundingSphere(bounding_sphere);
  int x,y,z,x_min,x_max,y_min,y_max,z_min,z_max;
  double xw,yw,zw;
  btVector3 v;
	
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()-bounding_sphere.radius,bounding_sphere.center.y()-bounding_sphere.radius,	
              bounding_sphere.center.z()-bounding_sphere.radius, x_min,y_min,z_min);
  worldToGrid(bounding_sphere.center,bounding_sphere.center.x()+bounding_sphere.radius,bounding_sphere.center.y()+bounding_sphere.radius,	
              bounding_sphere.center.z()+bounding_sphere.radius, x_max,y_max,z_max);
	
  // 	ROS_INFO("xyz_min: %i %i %i xyz_max: %i %i %i",x_min,y_min,z_min,x_max,y_max,z_max);
	
  //voxels.reserve(30000);
//  for(x = x_min; x <= x_max; ++x)
//  {
//    for(y = y_min; y <= y_max; ++y)
//    {
//      for(z = z_min; z <= z_max; ++z)
//      {
//        gridToWorld(bounding_sphere.center,x,y,z,xw,yw,zw);
//        if(body.containsPoint(xw,yw,zw))
//        {
//          v.setX(xw);
//          v.setY(yw);
//          v.setZ(zw);
//          //ROS_INFO_STREAM(xw << " " << yw << " " << zw);
//          voxels.push_back(v);
//        }
//      }
//    }
//  }


  for(x = x_min; x <= x_max; ++x)
  {
    for(y = y_min; y <= y_max; ++y)
    {
      for(z = z_min; z <= z_max; ++z)
      {
        gridToWorld(bounding_sphere.center,x,y,z,xw,yw,zw);

        v.setX(xw);
        v.setY(yw);
        v.setZ(zw);
        // compute all intersections
        int count=0;
        std::vector<btVector3> pts;
        body.intersectsRay(v, btVector3(0, 0, 1), &pts, count);

        // if we have an odd number of intersections, we are inside
        if (pts.size() % 2 == 1)
          voxels.push_back(v);
      }
    }
  }


  // 	ROS_INFO("number of occupied voxels in bounding sphere: %i", voxels.size());
}

}
