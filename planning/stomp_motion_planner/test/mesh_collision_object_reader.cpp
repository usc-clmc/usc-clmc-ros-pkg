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

#include "mesh_collision_object_reader.h"
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#include <ros/ros.h>

namespace stomp_motion_planner
{

MeshCollisionObjectReader::MeshCollisionObjectReader()
{
}

MeshCollisionObjectReader::~MeshCollisionObjectReader()
{
}

bool MeshCollisionObjectReader::readMesh(const std::string& file_name, geometric_shapes_msgs::Shape& shape)
{
  Assimp::Importer importer;

  const aiScene* scene = importer.ReadFile(file_name,
        aiProcess_Triangulate            |
        aiProcess_JoinIdenticalVertices  |
        aiProcess_SortByPType);

  if( !scene)
  {
    ROS_ERROR("Could not read mesh from file %s", file_name.c_str());
    return false;
  }

  if (!scene->HasMeshes())
  {
    ROS_ERROR("No meshes found in file [%s]", file_name.c_str());
    return false;
  }

  // initialize the shape object
  shape.type = shape.MESH;
  shape.triangles.clear();
  shape.vertices.clear();

  if (!buildMesh(scene, scene->mRootNode, shape))
  {
    ROS_ERROR("Could not read mesh from file %s", file_name.c_str());
    return false;
  }

  return true;
}

// Mostly stolen from rviz, which was mostly stolen from gazebo
bool MeshCollisionObjectReader::buildMesh(const aiScene* scene, const aiNode* node, geometric_shapes_msgs::Shape& shape)
{
  if (!node)
  {
    return true;
  }

  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();

  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    int point_start = shape.vertices.size();

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
    {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;

      geometry_msgs::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      shape.vertices.push_back(point);
    }

    // Add the triangles
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
    {
      aiFace& face = input_mesh->mFaces[j];
      if (face.mNumIndices != 3)
      {
        ROS_ERROR("Mesh loading failed because a face has %d vertices.", face.mNumIndices);
        return false;
      }
      for (uint32_t k = 0; k < face.mNumIndices; ++k)
      {
        shape.triangles.push_back(face.mIndices[k] + point_start);
      }
    }

  }

  bool success = true;
  for (uint32_t i=0; i < node->mNumChildren; ++i)
  {
    if (!buildMesh(scene, node->mChildren[i], shape))
    {
      success = false;
      break;
    }
  }

  return success;
}

}
