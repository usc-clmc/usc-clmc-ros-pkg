#include <tabletop_segmenter/utilities.h>
#include <rosbag/bag.h>
#include<fstream>

void tabletop_segmenter::storeBag(sensor_msgs::PointCloud2::ConstPtr whole_cloud,
				  const std::vector<sensor_msgs::PointCloud2> &clusters,
				  sensor_msgs::CameraInfo::ConstPtr cam_info,
				  sensor_msgs::Image::ConstPtr recent_rgb)
{
  /* record and store a bag file */
  rosbag::Bag bag;
  char name[512];
  time_t rawtime;
  struct tm * timeinfo;
  
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime (name,512,"/tmp/tabletop_segmentation_%Y_%m_%d_%I_%M_%S.bag",timeinfo);
  bag.open( name, rosbag::bagmode::Write);
  
  bag.write("whole_cloud", ros::Time::now(), whole_cloud);
  int count=0;
  for ( size_t i=0; i<clusters.size(); ++i)
    {
      sprintf(name, "cluster_%d", count);
      bag.write(name, ros::Time::now(), clusters[i]);
      count++;
    }

  bag.write("cam_info", ros::Time::now(), cam_info);

  bag.write("rgb", ros::Time::now(), recent_rgb);

  bag.close();
}

void tabletop_segmenter::storeBag(sensor_msgs::PointCloud2::ConstPtr whole_cloud,
				  const std::vector<sensor_msgs::PointCloud2> &clusters,
				  sensor_msgs::CameraInfo::ConstPtr cam_info)
{
  /* record and store a bag file */
  rosbag::Bag bag;
  char name[512];
  time_t rawtime;
  struct tm * timeinfo;
  
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  strftime (name,512,"/tmp/tabletop_segmentation_%Y_%m_%d_%I_%M_%S.bag",timeinfo);
  bag.open( name, rosbag::bagmode::Write);
  
  bag.write("whole_cloud", ros::Time::now(), whole_cloud);
  int count=0;
  for ( size_t i=0; i<clusters.size(); ++i)
    {
      sprintf(name, "cluster_%d", count);
      bag.write(name, ros::Time::now(), clusters[i]);
      count++;
    }

  bag.write("cam_info", ros::Time::now(), cam_info);

  bag.close();
}


void tabletop_segmenter::convert_non_dense_2_dense(const pcl::PointCloud<Point> &in, 
						   pcl::PointCloud<Point> &out)
{
  if(in.is_dense){
    ROS_WARN("Cloud is already dense. No conversion needed.\n");
    out = in;
    return;
  }

 
  // filter out Nan points
  out.points.reserve(in.points.size());
  pcl::PointCloud<Point>::const_iterator it = in.begin();
  for (; it<in.end(); it++)
    {
      Point pt = *it;
      if(pt.x == pt.x)
	out.points.push_back(pt);
    }

  // initialise header of new cloud from old cloud 
  out.is_dense = true;
  out.header = in.header;    
  out.width = out.points.size();
  out.height = 1;
  out.sensor_origin_ = in.sensor_origin_;
  out.sensor_orientation_ = in.sensor_orientation_;
}

void tabletop_segmenter::convert_hull_to_ply(const pcl::PointCloud<Point> &in, 
					     const std::vector< pcl::Vertices > &polygons, 
					     const char* filename)
{
  std::ofstream ofs(filename);
  
  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;
  ofs << "element vertex " << in.points.size() << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "element face " << polygons.size() << std::endl;
  ofs << "property list uchar int vertex_index" << std::endl;
  ofs << "end_header" << std::endl;
  
  for(size_t i = 0; i < in.points.size(); ++i)
    {
      ofs << in.points[i].x << " " 
	  << in.points[i].y << " " 
	  << in.points[i].z << std::endl;
    }
	
   for(size_t i = 0; i < polygons.size(); ++i)
     {
       ofs <<"3 "<< polygons[i].vertices[0] 
	   << " " 
	   << polygons[i].vertices[1]
	   << " " 
	   << polygons[i].vertices[2] <<std::endl;        
     }
  
  ofs << std::endl;

}
