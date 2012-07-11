/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         generate_grasp_library.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/

#include <vector>
#include <string>

#include <ros/ros.h>
#include <grasp_template_planning/demonstration_parser.h>
#include <grasp_template_planning/grasp_demo_library.h>
#include <grasp_template_planning/GraspAnalysis.h>
#include <grasp_template_planning/grasp_planning_params.h>.h>

using namespace std;
using namespace grasp_template_planning;

int main(int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR_STREAM("You missed some arguments. The correct call is: " << "generate_grasp_library [grasp_demonstrations_path] "
          "[grasp_library_path] [demo_filename] ...");
    return -1;
  }

  ros::init(argc, argv, "generate_grasp_library");
  ros::NodeHandle n;

  GraspDemoLibrary grasp_lib(argv[1], argv[2]);

  vector<string> demo_files;
  if (string(argv[3]).compare("analyze.all.") == 0)
  {
    grasp_lib.getAllDemonstrationFilenames(demo_files);
    grasp_lib.removeLibraryFileFromFilesystem();

    ROS_DEBUG("Following grasp demonstrations will be parsed to library: ");
    for (vector<string>::const_iterator it = demo_files.begin();
        it != demo_files.end(); it++)
    {
      ROS_DEBUG_STREAM(*it << endl);
    }
  }
  else
  {
    demo_files.push_back(argv[3]);
  }

  for (vector<string>::const_iterator it = demo_files.begin(); it != demo_files.end(); it++)
  {
    ROS_DEBUG_STREAM("Loading demonstration file" << it->c_str());
    grasp_lib.loadDemonstration(it->c_str());

    ROS_DEBUG_STREAM("Parsing demonstration...");
    DemonstrationParser analyzer(grasp_lib);
    GraspAnalysis ana;
    analyzer.analyzeGrasp(ana);

    ROS_DEBUG_STREAM("Writing demonstration to library...");
    grasp_lib.addAnalysisToLib(ana);
  }

  ROS_INFO("Done generating grasp library.");
  return 0;
}
