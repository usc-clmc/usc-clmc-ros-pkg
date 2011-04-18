/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		dmp_library_io.h

 \author	Peter Pastor
 \date		Jan 23, 2011

 *********************************************************************/

#ifndef DMP_LIBRARY_IO_H_
#define DMP_LIBRARY_IO_H_

// system includes
#include <string>
#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <usc_utilities/param_server.h>
#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

// local includes

namespace skill_library
{

template<class DMPType, class MessageType>
  class DMPLibraryIO
  {

  public:

    typedef boost::shared_ptr<MessageType const> MessageTypeConstPtr;

    /*! Constructor
     */
    DMPLibraryIO() :
      initialized_(false) {};

    /*! Destructor
     */
    virtual ~DMPLibraryIO() {};

    /*!
     * @return
     */
    bool initialize(const std::string& absolute_library_directory_path);

    /*!
     * @return
     */
    bool isInitialized()
    {
      return initialized_;
    }

  private:

    /*!
     */
    bool initialized_;

    /*!
     */
    boost::filesystem::path absolute_library_directory_path_;
    bool checkForDirectory();

  };

template<class DMPType, class MessageType>
  bool DMPLibraryIO<DMPType, MessageType>::initialize(const std::string& absolute_library_directory_path)
  {
  absolute_library_directory_path_ = boost::filesystem::path(absolute_library_directory_path);
  try
  {
    boost::filesystem::create_directories(absolute_library_directory_path_);
  }
  catch (std::exception e)
  {
    ROS_ERROR("Library directory >%s< could not be created: %s.", absolute_library_directory_path_.filename().c_str(), e.what());
    return false;
  }
  return (initialized_ = true);
  }

template<class DMPType, class MessageType>
  bool DMPLibraryIO<DMPType, MessageType>::checkForDirectory()
  {

    return true;
  }

}

#endif /* DMP_LIBRARY_IO_H_ */
