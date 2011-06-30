/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include "test_collision_world.h"
#include <mapping_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include <LinearMath/btQuaternion.h>
#include <mapping_msgs/AttachedCollisionObject.h>
#include <geometric_shapes_msgs/Shape.h>



namespace stomp_motion_planner
{

TestCollisionWorld::TestCollisionWorld()
{
  object_in_map_pub_  = node_handle_.advertise<mapping_msgs::CollisionObject>("collision_object", 10);
  rviz_pub_  = node_handle_.advertise<visualization_msgs::Marker>("collision_markers", 10);
  attached_object_pub_  = node_handle_.advertise<mapping_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  ros::WallDuration(2.0).sleep(); // hack to wait for subscribers
}

TestCollisionWorld::~TestCollisionWorld()
{
}

void TestCollisionWorld::createObjectsFromParamServer(ros::NodeHandle& node_handle)
{
  XmlRpc::XmlRpcValue list;
  if (node_handle.getParam("meshes", list))
  {
    for (int32_t i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR("Parameter %s/meshes needs to be an array of structs", node_handle.getNamespace().c_str());
        return;
      }

      std::string name = list[i]["name"];
      std::string url = list[i]["url"];
      std::string frame = list[i]["frame"];

      std::vector<double> position, orientation, scale;
      readDoubleArray(list[i]["position"], position);
      readDoubleArray(list[i]["orientation"], orientation);
      readDoubleArray(list[i]["scale"], scale);

      createMeshObject(name, url, frame, position, orientation, scale);
    }
  }

  if (node_handle.getParam("cylinders", list))
  {
    createObjectList(list, geometric_shapes_msgs::Shape::CYLINDER);
  }
  if (node_handle.getParam("boxes", list))
  {
    createObjectList(list, geometric_shapes_msgs::Shape::BOX);
  }
  if (node_handle.getParam("spheres", list))
  {
    createObjectList(list, geometric_shapes_msgs::Shape::SPHERE);
  }

  // HACK
  if (node_handle.hasParam("attached_object"))
  {
    //add a cylinder into the collision space attached to the r_gripper_r_finger_tip_link
    mapping_msgs::AttachedCollisionObject att_object;
    att_object.link_name = "r_gripper_palm_link";
    //att_object.touch_links.push_back("r_gripper_palm_link");
    att_object.touch_links.push_back("r_gripper_r_finger_link");
    att_object.touch_links.push_back("r_gripper_l_finger_link");
    att_object.touch_links.push_back("r_gripper_l_finger_tip_link");
    att_object.touch_links.push_back("r_gripper_r_finger_tip_link");

    att_object.object.id = "attached_can";
    att_object.object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
    att_object.object.header.frame_id = "r_gripper_tool_frame";
    att_object.object.header.stamp = ros::Time::now();
    geometric_shapes_msgs::Shape object;
    object.type = geometric_shapes_msgs::Shape::CYLINDER;
    object.dimensions.resize(2);
    object.dimensions[0] = .03;
    object.dimensions[1] = 0.15;
    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.07;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    att_object.object.shapes.push_back(object);
    att_object.object.poses.push_back(pose);

    attached_object_pub_.publish(att_object);

  }

}

void TestCollisionWorld::createObjectList(XmlRpc::XmlRpcValue list, int type)
{
  for (int32_t i = 0; i < list.size(); ++i)
  {
    if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Parameter needs to be an array of structs");
      return;
    }

    std::string name = list[i]["name"];
    std::string frame = list[i]["frame"];

    std::vector<double> position, orientation, dimensions;
    readDoubleArray(list[i]["position"], position);
    readDoubleArray(list[i]["orientation"], orientation);
    readDoubleArray(list[i]["dimensions"], dimensions);

    createObject(type, name, frame, position, orientation, dimensions);
  }
}

void TestCollisionWorld::createObject(int type, const std::string& name, const std::string& frame,
                  const std::vector<double>& position, const std::vector<double>& orientation, const std::vector<double>& dimensions)
{
  geometric_shapes_msgs::Shape object;
  object.type = type;
  object.dimensions = dimensions;

  mapping_msgs::CollisionObject collision_object;
  collision_object.id = name;
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  collision_object.header.frame_id = frame;
  collision_object.header.stamp = ros::Time::now();

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];

  // convert the RPY orientation to quaternion
  btQuaternion q;
  q.setRPY(orientation[0], orientation[1], orientation[2]);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);
  object_in_map_pub_.publish(collision_object);

}

void TestCollisionWorld::createMeshObject(const std::string& name, const std::string& url, const std::string& frame,
                  const std::vector<double>& position, const std::vector<double>& orientation, const std::vector<double>& scale)
{
  std::string file_name;
  if (!urlToFileName(url, file_name))
  {
    ROS_ERROR("Failed to convert url to file name.");
    return;
  }

  geometric_shapes_msgs::Shape object;
  if (!mesh_reader.readMesh(file_name, object))
  {
    ROS_ERROR("Failed to read mesh");
    return;
  }

  // scale the mesh, because the collision map probably doesn't support mesh scaling
  for (int i=0; i<int(object.vertices.size()); ++i)
  {
    object.vertices[i].x *= scale[0];
    object.vertices[i].y *= scale[1];
    object.vertices[i].z *= scale[2];
  }

  mapping_msgs::CollisionObject collision_object;
  collision_object.id = name;
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  collision_object.header.frame_id = frame;
  collision_object.header.stamp = ros::Time::now();

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];

  // convert the RPY orientation to quaternion
  btQuaternion q;
  q.setRPY(orientation[0], orientation[1], orientation[2]);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  collision_object.shapes.push_back(object);
  collision_object.poses.push_back(pose);
  object_in_map_pub_.publish(collision_object);

  // publish the same object to rviz:
  visualization_msgs::Marker marker;
  marker.header = collision_object.header;
  marker.pose = pose;
  marker.ns = name;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_use_embedded_materials = true;
  marker.scale.x = scale[0];
  marker.scale.y = scale[1];
  marker.scale.z = scale[2];
  marker.mesh_resource = url;
  rviz_pub_.publish(marker);

}

void TestCollisionWorld::createBookshelf()
{
  // read the mesh
  geometric_shapes_msgs::Shape object;
  if (!mesh_reader.readMesh("/home/kalakris/workspace/stomp_motion_planner/test/meshes/cabnite.dae", object))
  {
    return;
  }

  mapping_msgs::CollisionObject bookshelf_object;
  bookshelf_object.id = "bookshelf";
  bookshelf_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  bookshelf_object.header.frame_id = "/base_footprint";
  bookshelf_object.header.stamp = ros::Time::now();
  geometry_msgs::Pose pose;
  pose.position.x = 0;
  pose.position.y = 0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  bookshelf_object.shapes.push_back(object);
  bookshelf_object.poses.push_back(pose);
  object_in_map_pub_.publish(bookshelf_object);
}

void TestCollisionWorld::createPole()
{
  mapping_msgs::CollisionObject cylinder_object;
  cylinder_object.id = "pole";
  cylinder_object.operation.operation = mapping_msgs::CollisionObjectOperation::ADD;
  cylinder_object.header.frame_id = "base_link";
  cylinder_object.header.stamp = ros::Time::now();
  geometric_shapes_msgs::Shape object;
  object.type = geometric_shapes_msgs::Shape::CYLINDER;
  object.dimensions.resize(2);
  object.dimensions[0] = .1;
  object.dimensions[1] = 1.2;
  geometry_msgs::Pose pose;
  pose.position.x = .65;
  pose.position.y = -.65;
  pose.position.z = .6;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  cylinder_object.shapes.push_back(object);
  cylinder_object.poses.push_back(pose);
  object_in_map_pub_.publish(cylinder_object);
}

void TestCollisionWorld::removeAll()
{
  mapping_msgs::CollisionObject collision_object;
  collision_object.id = "all";
  collision_object.operation.operation = mapping_msgs::CollisionObjectOperation::REMOVE;
  collision_object.header.frame_id = "/base_footprint";
  collision_object.header.stamp = ros::Time::now();
  object_in_map_pub_.publish(collision_object);
}

bool TestCollisionWorld::readDoubleArray(XmlRpc::XmlRpcValue& list, std::vector<double>& array)
{
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("Parameter needs to be an *array* of doubles");
        return false;
    }

    array.clear();
    for (int32_t i = 0; i < list.size(); ++i)
    {
        if (list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
                list[i].getType() != XmlRpc::XmlRpcValue::TypeInt)
        {
            ROS_ERROR("Parameter needs to be an array of *doubles*");
            return false;
        }
        double value=0.0;
        if (list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
            value = static_cast<double>(static_cast<int>(list[i]));
        else value = static_cast<double>(list[i]);
        array.push_back(value);
    }
    return true;
}

bool TestCollisionWorld::urlToFileName(const std::string& url, std::string& file_name)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      ROS_ERROR("Could not parse url %s", url.c_str());
      return false;
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      ROS_ERROR("Package [%s] does not exist", package.c_str());
      return false;
    }

    file_name = package_path + mod_url;
  }
  else if (url.find("file://") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    file_name = mod_url;
  }
  else
  {
    ROS_ERROR("Could not parse url %s", url.c_str());
    return false;
  }
  return true;
}

}
