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

#ifndef TEST_COLLISION_WORLD_H_
#define TEST_COLLISION_WORLD_H_

#include <ros/ros.h>
#include "mesh_collision_object_reader.h"

namespace stomp_motion_planner
{

class TestCollisionWorld
{
public:
  TestCollisionWorld();
  virtual ~TestCollisionWorld();

  void createPole();
  void createBookshelf();
  void createObjectsFromParamServer(ros::NodeHandle& node_handle);
  void createMeshObject(const std::string& name, const std::string& url, const std::string& frame,
                    const std::vector<double>& position, const std::vector<double>& orientation, const std::vector<double>& scale);
  void createObject(int type, const std::string& name, const std::string& frame,
                    const std::vector<double>& position, const std::vector<double>& orientation, const std::vector<double>& dimensions);
  void createObjectList(XmlRpc::XmlRpcValue list, int type);
  void removeAll();

private:
  ros::NodeHandle node_handle_;
  ros::Publisher object_in_map_pub_;
  ros::Publisher attached_object_pub_;
  ros::Publisher rviz_pub_;
  MeshCollisionObjectReader mesh_reader;

  bool readDoubleArray(XmlRpc::XmlRpcValue& list, std::vector<double>& array);
  bool urlToFileName(const std::string& url, std::string& file_name);

};

}

#endif /* TEST_COLLISION_WORLD_H_ */
