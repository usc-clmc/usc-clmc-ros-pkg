
#include <pcl/surface/convex_hull.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <omp.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{


string cloud_filename = "flashlight1.pcd";

PointCloud<PointXYZRGB>::Ptr image_A_unf(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr image_A(new PointCloud<PointXYZRGB>);

assert(pcl::io::loadPCDFile<PointXYZRGB> (cloud_filename, *image_A_unf) != -1);

pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (image_A_unf);
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*image_A);

#pragma omp parallel shared(image_A)
{

	  std::vector<pcl::Vertices> convex_hull_vertices_gl;
	  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > convex_hull_points_gl;

	  {
		  ConvexHull < PointXYZRGB > cv;
		  cv.setInputCloud(image_A);
		  boost::shared_ptr < PointCloud<PointXYZRGB> > cv_points;
		  cv_points.reset(new PointCloud<PointXYZRGB> ());

		  cv.reconstruct(*cv_points, convex_hull_vertices_gl);

		  convex_hull_points_gl = cv_points;
	  }

	while(true)
	{
	  ConvexHull < PointXYZRGB > cv;
	  cv.setInputCloud(image_A);
	  boost::shared_ptr < PointCloud<PointXYZRGB> > cv_points;
	  cv_points.reset(new PointCloud<PointXYZRGB> ());

	  std::vector<pcl::Vertices> convex_hull_vertices_;
	  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > convex_hull_points_;
	  cv.reconstruct(*cv_points, convex_hull_vertices_);

	  convex_hull_points_ = cv_points;

	  //test if equal
	  for (unsigned int i = 0; i < convex_hull_vertices_gl.size(); i++)
	  {
		  if(convex_hull_vertices_gl[i].vertices[0] != convex_hull_vertices_[i].vertices[0])
			  ROS_INFO("ERROR");
		  if(convex_hull_vertices_gl[i].vertices[1] != convex_hull_vertices_[i].vertices[1])
			  ROS_INFO("ERROR");
		  if(convex_hull_vertices_gl[i].vertices[2] != convex_hull_vertices_[i].vertices[2])
			  ROS_INFO("ERROR");
	  }

	  for (unsigned int i = 0; i < convex_hull_points_->points.size(); i++)
	  {
		  if(convex_hull_points_gl->points[i].x != convex_hull_points_->points[i].x)
			  ROS_INFO("ERROR");

		  if(convex_hull_points_gl->points[i].y != convex_hull_points_->points[i].y)
			  ROS_INFO("ERROR");

		  if(convex_hull_points_gl->points[i].z != convex_hull_points_->points[i].z)
			  ROS_INFO("ERROR");

	  }

	  ROS_INFO("Fine");
	}
}


	return 0;
}
