/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dmp_library.h

  \author	Peter Pastor
  \date		Jan 22, 2011

 *********************************************************************/

#ifndef DMP_LIBRARY_H_
#define DMP_LIBRARY_H_

// system includes
#include <string>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

// local includes
#include <skill_library/dmp_library_io.h>

namespace skill_library
{

template<class DMPType, class MessageType>
class DMPLibrary
{

public:

  /*! Constructor
   */
  DMPLibrary() :
    initialized_(false) {};

  /*! Destructor
   */
  virtual ~DMPLibrary() {};

  /*!
   * @param data_directory_name
   * @return
   */
  bool initialize(const std::string& data_directory_name);

  /*!
   *
   * @param name
   * @param dmp_message
   * @return
   */
  bool getDMP(const std::string& name,
              MessageType& dmp_message);

  /*!
   *
   * @param name
   * @param dmp_message
   * @return
   */
  bool addDMP(const MessageType& dmp_message,
              const std::string& name);

  /*!
   *
   * @param name
   * @param dmp_message
   * @return
   */
  bool addDMP(const typename DMPType::DMPPtr& dmp,
              const std::string& name);

    /*!
     * @param file_name
     * @return
     */
    std::string getBagFileName(const std::string& file_name)
    {
      return absolute_library_directory_path_.file_string() + "/" + file_name + ".bag";
    }

  private:

    /*!
     */
    bool initialized_;

    /*!
     */
    boost::filesystem::path absolute_library_directory_path_;

};

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::initialize(const std::string& data_directory_name)
  {
    std::string library_directory_name = data_directory_name + DMPType::getVersionString();
    absolute_library_directory_path_ = boost::filesystem::path(library_directory_name);
    ROS_INFO("Initializing DMP library with path >%s<.", absolute_library_directory_path_.file_string().c_str());
    try
    {
      boost::filesystem::create_directories(absolute_library_directory_path_);
    }
    catch (std::exception e)
    {
      ROS_ERROR("Library directory >%s< could not be created: %s.", absolute_library_directory_path_.file_string().c_str(), e.what());
      return false;
    }
    return (initialized_ = true);
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::addDMP(const MessageType& dmp_message,
                                                const std::string& name)
  {
  if(name.empty())
  {
    ROS_ERROR("Cannot add DMP without name. Name must be specified.");
    return false;
  }
    ROS_INFO("Writing into DMP Library at >%s<.", getBagFileName(name).c_str());
    return dmp::DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(dmp_message, getBagFileName(name));
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::addDMP(const typename DMPType::DMPPtr& dmp,
                                                const std::string& name)
  {
    if (name.empty())
    {
      ROS_ERROR("Cannot add DMP without name. Name must be specified.");
      return false;
    }
    ROS_INFO("Writing into DMP Library at >%s<.", getBagFileName(name).c_str());
    return dmp::DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(dmp, getBagFileName(name));
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::getDMP(const std::string& name,
                                                MessageType& dmp_message)
  {
  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for (boost::filesystem::directory_iterator itr(absolute_library_directory_path_); itr != end_itr; ++itr)
  {
    ROS_INFO("checking: %s and %s", itr->path().file_string().c_str(), getBagFileName(name).c_str());
    if(getBagFileName(name).compare(itr->path().file_string()) == 0)
    {
      return usc_utilities::FileIO<MessageType>::readFromBagFile(dmp_message, DMPType::getVersionString(), getBagFileName(name));
    }
  }
    ROS_ERROR("Could not find DMP with name >%s<", name.c_str());
    return false;
  }
}

#endif /* DMP_LIBRARY_H_ */
