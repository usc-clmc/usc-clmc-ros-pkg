/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         visualization.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <cassert>
#include <Eigen/Eigen>

#include <pcl/Vertices.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <grasp_template/dismatch_measure.h>
#include <grasp_template_planning/visualization.h>
#include <sensor_msgs/point_cloud_conversion.h>

using namespace std;
using namespace grasp_template;
using namespace visualization_msgs;
using namespace Eigen;

namespace grasp_template_planning
{

Visualization::Visualization(bool slim_publishing, double hz) :
  rate_(hz), slim_publishing_(slim_publishing)
{
}

Visualization::~Visualization()
{
  stopPublishing();
}

void Visualization::initialize(ros::NodeHandle& n)
{
  boost::mutex::scoped_lock lock(mutex_);

  /* create publisher */
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_hull",
      n.advertise<sensor_msgs::PointCloud> ("grasp_target_hull", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_search_points",
      n.advertise<sensor_msgs::PointCloud> ("grasp_search_points", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_hull_mesh",
      n.advertise<Marker> ( "grasp_target_hull_mesh", 50)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_matching_object",
      n.advertise<sensor_msgs::PointCloud> ("grasp_matching_object", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_object",
      n.advertise<sensor_msgs::PointCloud> ("grasp_target_object", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_matching_gripper",
      n.advertise<geometry_msgs::PoseStamped> ("grasp_matching_gripper", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_gripper",
      n.advertise<geometry_msgs::PoseStamped> ("grasp_target_gripper", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_matching_templt",
      n.advertise<Marker> ("grasp_matching_templt", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_templt",
      n.advertise<Marker> ("grasp_target_templt", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_templt_comprsn_m",
      n.advertise<Marker> ("grasp_templt_comprsn_m", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_templt_comprsn_t",
      n.advertise<Marker> ("grasp_templt_comprsn_t", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_normals",
      n.advertise<Marker> ("grasp_target_normals", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_gripper_mesh",
      n.advertise<Marker> ("grasp_target_gripper_mesh", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_matching_gripper_mesh",
      n.advertise<Marker> ("grasp_matching_gripper_mesh", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_target_templt_pose",
      n.advertise<geometry_msgs::PoseStamped> ("grasp_target_templt_pose", 5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("grasp_viewpoint_pose",
      n.advertise<geometry_msgs::PoseStamped> ("grasp_viewpoint_pose",5)));
  publisher_.insert(make_pair<string, ros::Publisher> ("ghm_table_pose",
      n.advertise<geometry_msgs::PoseStamped> ("ghm_table_pose",5)));
}

void Visualization::resetData(const PlanningPipeline& dp, const GraspAnalysis& ref,
    const GraspAnalysis& trgt_templt)
{
  boost::mutex::scoped_lock lock(mutex_);

  /* target_normals */
  string frame_id = dp.target_object_.header.frame_id;
  target_normals_ = dp.templt_generator_->getVisualizationNormals("target_obj_normals_viz", frame_id);
  /* viewpoint */
  viewpoint_pose_.header.stamp = ros::Time::now();
  viewpoint_pose_.header.frame_id = dp.templt_generator_->frameBase();

  viewpoint_pose_.pose.orientation.w = dp.templt_generator_->viewp_rot_.w();
  viewpoint_pose_.pose.orientation.x = dp.templt_generator_->viewp_rot_.x();
  viewpoint_pose_.pose.orientation.y = dp.templt_generator_->viewp_rot_.y();
  viewpoint_pose_.pose.orientation.z = dp.templt_generator_->viewp_rot_.z();

  viewpoint_pose_.pose.position.x = dp.templt_generator_->viewp_trans_.x();
  viewpoint_pose_.pose.position.y = dp.templt_generator_->viewp_trans_.y();
  viewpoint_pose_.pose.position.z = dp.templt_generator_->viewp_trans_.z();


  table_pose_.header.stamp = ros::Time::now();
  table_pose_.header.frame_id = dp.templt_generator_->frameBase();

  table_pose_.pose = dp.templt_generator_->table_pose_;

  sensor_msgs::PointCloud2 pc2_tmp;
  pcl::toROSMsg(dp.templt_generator_->getConvexHullPoints(), pc2_tmp);
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_tmp, target_hull_);
  computeHullMesh(dp.templt_generator_->getConvexHullPoints(), dp.templt_generator_->getConvexHullVertices());

  pcl::toROSMsg(dp.templt_generator_->getSearchPoints(), pc2_tmp);
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_tmp, search_points_);

  sensor_msgs::ChannelFloat32 sp_intens;
  sp_intens.name = "intensity";
  sp_intens.values.resize(search_points_.points.size());
  for (unsigned int i = 0; i < search_points_.points.size(); i++)
  {
    sp_intens.values[i] = i;
  }
  search_points_.channels.push_back(sp_intens);

  /* target object */
  sensor_msgs::convertPointCloud2ToPointCloud(dp.target_object_, target_object_);

  /* reference object */
  dp.getRelatedObject(ref, pc2_tmp);
  sensor_msgs::convertPointCloud2ToPointCloud(pc2_tmp, matching_object_);

  /* target and matching gripper-poses */
  matching_gripper_ = ref.gripper_pose;
  target_gripper_ = trgt_templt.gripper_pose;

  /* target template */
  GraspTemplate t(ref.grasp_template, ref.template_pose.pose);
  DismatchMeasure match_handl(t, matching_gripper_.pose);
  GraspTemplate current_candidate(trgt_templt.grasp_template, trgt_templt.template_pose.pose);
  match_handl.applyDcMask(current_candidate);
  target_templt_ = current_candidate.getVisualization("grasp_decision_target_viz", target_object_.header.frame_id);

  /* target templt */
  target_templt_pose_.header.stamp = ros::Time::now();
  target_templt_pose_.header.frame_id = target_object_.header.frame_id;

  target_templt_pose_.pose.position.x = current_candidate. object_to_template_translation_.x();
  target_templt_pose_.pose.position.y = current_candidate. object_to_template_translation_.y();
  target_templt_pose_.pose.position.z = current_candidate. object_to_template_translation_.z();

  target_templt_pose_.pose.orientation.x = current_candidate. object_to_template_rotation_.x();
  target_templt_pose_.pose.orientation.y = current_candidate. object_to_template_rotation_.y();
  target_templt_pose_.pose.orientation.z = current_candidate. object_to_template_rotation_.z();
  target_templt_pose_.pose.orientation.w = current_candidate. object_to_template_rotation_.w();

  /* target heightmap */
  target_hm_
      = current_candidate.heightmap_.getVisualization("templt_comp_target", ref.template_pose.header.frame_id, 1);

  /* matching template */

  matching_templt_ = match_handl.getLibTemplt().getVisualization("grasp_decision_match_viz",
                                                                 ref.template_pose.header.frame_id);
  /* matching heightmap */
  matching_hm_ = t.heightmap_.getVisualization("templt_comp_match", ref.template_pose.header.frame_id);

  /* transform matchings for better visualization */
  Vector3d viz_trans(0, -0.5, 0.5);
  Quaterniond viz_rot = Quaterniond::Identity();
  transformMarkers(matching_templt_, viz_trans, viz_rot);
  transformPose(matching_gripper_.pose, viz_trans, viz_rot);
  transformPointCloud(matching_object_, viz_trans, viz_rot);

  viz_trans.x() = matching_gripper_.pose.position.x;
  viz_trans.y() = matching_gripper_.pose.position.y + 0.35;
  viz_trans.z() = matching_gripper_.pose.position.z - 0.2;

  adaptForTemplateComparison(matching_hm_, target_hm_, viz_trans, Quaterniond::Identity());

  /* visualize gripper in form of a mesh */
  string gripper_arch;
  double gripper_state = 0;
  ros::param::get("~hand_architecture", gripper_arch);
  if (gripper_arch == "armrobot" && ref.fingerpositions.vals.size() >= 1)
  {
    gripper_state = ref.fingerpositions.vals[0];
  }
  else
  {
    gripper_state = ref.gripper_joint_state;
  }
  matching_gripper_mesh_ = visualizeGripper("matching_gripper_mesh",
      matching_object_.header.frame_id, matching_gripper_.pose, gripper_state);
  target_gripper_mesh_ = visualizeGripper("target_gripper_mesh",
      target_object_.header.frame_id, target_gripper_.pose, gripper_state);
}

void Visualization::transformMarkers(vector<Marker>& container, const Vector3d& trans,
                                     const Quaterniond& rot)
{
  for (vector<Marker>::iterator vec_it = container.begin();
      vec_it != container.end(); vec_it++)
  {
    for (vector<geometry_msgs::Point>::iterator p_it = vec_it->points.begin();
        p_it != vec_it->points.end(); p_it++)
    {
      Vector3d pos;
      pos.x() = p_it->x;
      pos.y() = p_it->y;
      pos.z() = p_it->z;

      pos = rot * pos + trans;

      p_it->x = pos.x();
      p_it->y = pos.y();
      p_it->z = pos.z();
    }
  }
}

void Visualization::transformPose(geometry_msgs::Pose& pose,
    const Vector3d& trans, const Quaterniond& rot)
{
  Vector3d pos;
  pos.x() = pose.position.x;
  pos.y() = pose.position.y;
  pos.z() = pose.position.z;

  pos = rot * pos + trans;

  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();

  Quaterniond orientation;
  orientation.x() = pose.orientation.x;
  orientation.y() = pose.orientation.y;
  orientation.z() = pose.orientation.z;
  orientation.w() = pose.orientation.w;

  orientation = rot * orientation;

  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
}

void Visualization::transformPointCloud(sensor_msgs::PointCloud& pc,
    const Vector3d& trans, const Quaterniond& rot)
{
  for (vector<geometry_msgs::Point32>::iterator p_it = pc.points.begin(); p_it != pc.points.end(); p_it++)
  {
    Vector3d pos;
    pos.x() = p_it->x;
    pos.y() = p_it->y;
    pos.z() = p_it->z;

    pos = rot * pos + trans;

    p_it->x = pos.x();
    p_it->y = pos.y();
    p_it->z = pos.z();
  }
}

void Visualization::adaptForTemplateComparison(vector<Marker>& m1,
    vector<Marker>& m2, const Vector3d& trans, const Quaterniond& rot)
{
  transformMarkers(m1, trans, rot);
  transformMarkers(m2, trans, rot);

  m1[0].color.g = 1.0;
  m1[0].color.r = 0.0;
  m2[0].color.b = 1.0;
  m2[0].color.r = 0.0;
}

void Visualization::startPublishing()
{
  stop_requested_ = false;
  if (publish_thread_ != NULL)
  {
    ROS_DEBUG("grasp_template_planning::Visualization: Thread pointer is not NULL.");
  }
  publish_thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&Visualization::doPublishing, this)));
}

void Visualization::stopPublishing()
{
  stop_requested_ = true;
  if (publish_thread_ != NULL)
    publish_thread_->join();
}

void Visualization::doPublishing()
{
  while (ros::ok() && !stop_requested_)
  {
    boost::mutex::scoped_lock lock(mutex_);

    publishOnce();

    lock.unlock();
    rate_.sleep();
  }
}

void Visualization::publishOnce()
{
  publisher_.find("grasp_matching_gripper")->second.publish(matching_gripper_);
  publisher_.find("grasp_target_gripper")->second.publish(target_gripper_);
  publisher_.find("grasp_viewpoint_pose")->second.publish(viewpoint_pose_);
  publisher_.find("ghm_table_pose")->second.publish(table_pose_);
  publisher_.find("grasp_matching_object")->second.publish(matching_object_);
  publisher_.find("grasp_target_object")->second.publish(target_object_);
  for (vector<Marker>::const_iterator it = target_gripper_mesh_.begin();
      it != target_gripper_mesh_.end(); it++)
  {
    publisher_.find("grasp_target_gripper_mesh")->second.publish(*it);
  }

  if (slim_publishing_)
  {
    return;
  }
  publisher_.find("grasp_target_hull")->second.publish(target_hull_);
  publisher_.find("grasp_target_templt_pose")->second.publish(target_templt_pose_);
  for (vector<Marker>::const_iterator it = matching_gripper_mesh_.begin(); it != matching_gripper_mesh_.end(); it++)
  {
    publisher_.find("grasp_matching_gripper_mesh")->second.publish(*it);
  }
  for (vector<Marker>::const_iterator it = matching_templt_.begin(); it != matching_templt_.end(); it++)
  {
    publisher_.find("grasp_matching_templt")->second.publish(*it);
  }
  for (vector<Marker>::const_iterator it = target_templt_.begin(); it != target_templt_.end(); it++)
  {
    publisher_.find("grasp_target_templt")->second.publish(*it);
  }
  for (vector<Marker>::const_iterator it = matching_hm_.begin(); it != matching_hm_.end(); it++)
  {
    publisher_.find("grasp_templt_comprsn_m")->second.publish(*it);
  }
  for (vector<Marker>::const_iterator it = target_hm_.begin(); it != target_hm_.end(); it++)
  {
    publisher_.find("grasp_templt_comprsn_t")->second.publish(*it);
  }
  for (vector<Marker>::const_iterator it = target_hull_mesh_.begin(); it != target_hull_mesh_.end(); it++)
  {
    publisher_.find("grasp_target_hull_mesh")->second.publish(*it);
  }

  publisher_.find("grasp_target_normals")->second.publish(target_normals_);
  publisher_.find("grasp_search_points")->second.publish(search_points_);
}

Marker Visualization::visualizeGripperPart(const string& resource, const string& ns,
                                           const string& frame_id, int id) const
{
  Marker mesh;
  mesh.header.frame_id = frame_id;
  mesh.header.stamp = ros::Time::now();
  mesh.ns = ns;
  mesh.id = id;
  mesh.type = Marker::MESH_RESOURCE;
  mesh.action = Marker::ADD;
  mesh.mesh_use_embedded_materials = true;

  mesh.scale.x = 1;
  mesh.scale.y = 1;
  mesh.scale.z = 1;

  mesh.mesh_resource = resource;

  return mesh;
}

Marker Visualization::visualizeGripperBox(const string& ns, const string& frame_id, int id) const
{
  Marker line_list;

  line_list.header.frame_id = frame_id;
  line_list.header.stamp = ros::Time::now();

  line_list.ns = ns;
  line_list.id = id;//1

  line_list.type = Marker::LINE_LIST;
  line_list.action = Marker::ADD;

  line_list.pose.orientation.w = 1.0;
  line_list.scale.x = 0.001;
  line_list.color.g = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point e1, e2;
  {
    GraspTemplateParams templt_hlpr;
    Vector3d v1, v2;
    templt_hlpr.actuatorBoundingBox(v1, v2);
    e1.x = v1.x();
    e1.y = v1.y();
    e1.z = v1.z();

    e2.x = v2.x();
    e2.y = v2.y();
    e2.z = v2.z();
  }
  double dx = abs(e2.x - e1.x);
  double dy = abs(e2.y - e1.y);
  double dz = abs(e2.z - e1.z);

  {
    line_list.points.push_back(e1);

    geometry_msgs::Point p = e1;
    p.x += dx;

    line_list.points.push_back(p);
  }
  {
    line_list.points.push_back(e1);

    geometry_msgs::Point p = e1;
    p.y += dy;

    line_list.points.push_back(p);
  }
  {
    line_list.points.push_back(e1);

    geometry_msgs::Point p = e1;
    p.z += dz;

    line_list.points.push_back(p);
  }

  {
    line_list.points.push_back(e2);

    geometry_msgs::Point p = e2;
    p.x -= dx;

    line_list.points.push_back(p);
  }
  {
    line_list.points.push_back(e2);

    geometry_msgs::Point p = e2;
    p.y -= dy;

    line_list.points.push_back(p);
  }
  {
    line_list.points.push_back(e2);

    geometry_msgs::Point p = e2;
    p.z -= dz;

    line_list.points.push_back(p);
  }

  return line_list;
}

vector<Marker> Visualization::visualizeGripper(const string& ns,
    const string& frame_id, const geometry_msgs::Pose& pose, double gripper_state) const
{
  Vector3d trans(pose.position.x, pose.position.y, pose.position.z);
  Quaterniond rot(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  Marker l_finger = visualizeGripperPart("package://pr2_description/meshes/gripper_v0"
    "/l_finger_tip.dae", ns, frame_id, 0);
  Marker r_finger = visualizeGripperPart("package://pr2_description/meshes/gripper_v0"
    "/l_finger_tip.dae", ns, frame_id, 1);
  Marker palm;
  palm = visualizeGripperPart("package://pr2_description/meshes/gripper_v0"
    "/gripper_palm.dae", ns, frame_id, 2);

  Marker bounding_box = visualizeGripperBox("bounding_box", frame_id, 4);
  bounding_box.pose = pose;

  Vector3d position(-0.051, 0, -0.025);
  Quaterniond rotation = Quaterniond::Identity();
  position = rot * position + trans;
  rotation = rotation * rot;
  palm.pose.position.x = position.x();
  palm.pose.position.y = position.y();
  palm.pose.position.z = position.z();
  palm.pose.orientation.w = rotation.w();
  palm.pose.orientation.x = rotation.x();
  palm.pose.orientation.y = rotation.y();
  palm.pose.orientation.z = rotation.z();

  double finger_y_delta = gripper_state * 0.8 / 2;
  double finger_x_delta = 0;

  position = Vector3d(0.1175 - finger_x_delta, 0.015 + finger_y_delta, -0.025);
  rotation = Quaterniond::Identity();
  position = rot * position + trans;
  rotation = rotation * rot;
  l_finger.pose.position.x = position.x();
  l_finger.pose.position.y = position.y();
  l_finger.pose.position.z = position.z();
  l_finger.pose.orientation.w = rotation.w();
  l_finger.pose.orientation.x = rotation.x();
  l_finger.pose.orientation.y = rotation.y();
  l_finger.pose.orientation.z = rotation.z();

  position = Vector3d(0.1175 - finger_x_delta, -0.015 - finger_y_delta, -0.025);
  rotation = Quaterniond(AngleAxisd(M_PI, Vector3d(1, 0, 0)));
  position = rot * position + trans;
  rotation = rot * rotation;
  r_finger.pose.position.x = position.x();
  r_finger.pose.position.y = position.y();
  r_finger.pose.position.z = position.z();
  r_finger.pose.orientation.w = rotation.w();
  r_finger.pose.orientation.x = rotation.x();
  r_finger.pose.orientation.y = rotation.y();
  r_finger.pose.orientation.z = rotation.z();

  vector < Marker > result;
  result.push_back(palm);
  result.push_back(l_finger);
  result.push_back(r_finger);

  /* palm marker */
  getTemplateExtractionPoint(position);
  rotation = Quaterniond::Identity();
  position = rot * position + trans;
  rotation = rotation * rot;

  Marker palm_marker;
  palm_marker.header.frame_id = frame_id;
  palm_marker.header.stamp = ros::Time::now();
  palm_marker.ns = ns;
  palm_marker.id = 3;
  palm_marker.type = Marker::SPHERE;
  palm_marker.action = Marker::ADD;
  palm_marker.pose.position.x = position.x();
  palm_marker.pose.position.y = position.y();
  palm_marker.pose.position.z = position.z();
  palm_marker.pose.orientation.x = 0.0;
  palm_marker.pose.orientation.y = 0.0;
  palm_marker.pose.orientation.z = 0.0;
  palm_marker.pose.orientation.w = 1.0;
  palm_marker.scale.x = 0.01;
  palm_marker.scale.y = 0.01;
  palm_marker.scale.z = 0.01;
  palm_marker.color.a = 1.0;
  palm_marker.color.r = 0.0;
  palm_marker.color.g = 1.0;

  result.push_back(palm_marker);
  return result;
}

void Visualization::computeHullMesh(const pcl::PointCloud<pcl::PointXYZ>& points,
                                    const vector<pcl::Vertices>& vertices)
{
  target_hull_mesh_.clear();

  Marker mesh;
  mesh.header.frame_id = points.header.frame_id;
  mesh.header.stamp = ros::Time::now();
  mesh.ns = "grasp_target_hull_viz";
  mesh.id = 30;
  mesh.type = Marker::LINE_LIST;
  mesh.action = Marker::ADD;
  mesh.scale.x = 0.001;
  mesh.pose.orientation.w = 1.0;
  mesh.color.r = 1.0;
  mesh.color.a = 1.0;

  geometry_msgs::Point first, prev, cur;
  bool is_first = true;

  for (unsigned int i = 0; i < vertices.size(); i++)
  {
    is_first = true;
    for (unsigned int j = 0; j < vertices[i].vertices.size(); j++)
    {
      unsigned int index = vertices[i].vertices[j];

      if (index >= points.points.size())
      {
        assert(false);
        continue;
      }

      prev = cur;
      cur.x = points.points[index].x;
      cur.y = points.points[index].y;
      cur.z = points.points[index].z;

      if (is_first)
      {
        first = cur;
        is_first = false;
      }
      else
      {
        mesh.points.push_back(prev);
        mesh.points.push_back(cur);
      }
    }

    mesh.points.push_back(cur);
    mesh.points.push_back(first);
  }

  target_hull_mesh_.push_back(mesh);
}

} //namespace
