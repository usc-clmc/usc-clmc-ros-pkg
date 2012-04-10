/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		cross_validator_node.cpp

  \author	Peter Pastor
  \date		Jul 5, 2011

 *********************************************************************/

// system includes
#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>

// local includes
#include <task_event_detector/modelselection_grid_search_kernel.h>
#include <task_event_detector/shogun_init.h>

int main(int argc, char **argv)
{
  if(argc < 2)
  {
    printf("ERROR: No node name provided. %i\n", argc);
    for(int i=0; i<argc; i++)
    {
      printf("argv[%i] = %s\n", i, argv[i]);
    }
    return -1;
  }
  printf("Initializing node named: >%s<.\n", argv[1]);
  ros::init(argc, argv, argv[1]);
  ros::NodeHandle node_handle("~");
//  task_event_detector::init();
//  task_event_detector::ModelSelectionGridSearchKernel cross_validator(node_handle);
//  task_event_detector::exit();
  printf("Exiting node named: >%s<.\n", argv[1]);
  return 0;
}
