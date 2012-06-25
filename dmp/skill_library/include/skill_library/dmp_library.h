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
#include <map>
#define BOOST_FILESYSTEM_VERSION 2
#include <boost/filesystem.hpp>
#include <algorithm>

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

static const std::string SLASH = "/";
static const std::string DESCRIPTION_ID_SEPARATOR = "_";
static const std::string BAG_FILE_ENDING = ".bag";

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

    /*! Retreives the DMP from the library
     * @param name Name of the DMP. Must be of the form <description_id>
     * @param dmp_message
     * @return
     */
    bool getDMP(const std::string& name,
                MessageType& dmp_message);

    /*! Adds DMP to library and sets the DMP id.
     * It also appends the id to the name.
     * @param name Name of the DMP. Name of the DMP. Must be of the form <description>
     * @param dmp_message
     * @return
     */
    bool addDMP(MessageType& dmp_message,
                std::string& name);

    /*! Reloads all DMPs from disc into a buffer
     * @return True on success, otherwise False
     */
    bool reload();

    /*!
     * @return
     */
    bool print();

    /*!
     * @param name
     * @return converted absolute bag file name
     */
    std::string getBagFileName(const std::string& name)
    {
      return absolute_library_directory_path_.file_string() + SLASH + name + BAG_FILE_ENDING;
    }

  private:

    std::string removeBagFileEnding(const std::string& filename)
    {
      std::string name = filename;
      size_t bag_separater_pos = name.rfind(BAG_FILE_ENDING);
      if(bag_separater_pos != std::string::npos && name.length() >= BAG_FILE_ENDING.length())
      {
        name = name.substr(0, bag_separater_pos);
      }
      return name;
    }

    /*! Gets the local filename from the absolute filename and removes bag file endings
     * @param absolute_filename
     * @param dmp_name
     * @return
     */
    std::string getName(const std::string& absolute_filename)
    {
      std::string filename = absolute_filename;
      std::string local_filename = filename;
      size_t slash_separater_pos = filename.rfind(SLASH);
      if (slash_separater_pos != std::string::npos)
      {
        size_t sp = slash_separater_pos + SLASH.length();
        size_t length = filename.length() - sp;
        local_filename = filename.substr(sp, length);
      }
      return removeBagFileEnding(local_filename);
    }

    /*! Parses DMP name into <description> and <id>
     * @param absolute_filename
     * @param name
     * @param id
     * @return True on success, otherwise False
     */
    bool parseName(const std::string& absolute_filename, std::string& name, int& id)
    {
      std::string filename = getName(absolute_filename);
      size_t separater_pos = filename.rfind(DESCRIPTION_ID_SEPARATOR);
      if (separater_pos != std::string::npos)
      {
        size_t sp = separater_pos + DESCRIPTION_ID_SEPARATOR.length();
        size_t length = filename.length() - sp;
        name = filename.substr(0, separater_pos);
        std::string id_string = filename.substr(sp, length);
        try
        {
          id = boost::lexical_cast<int>(id_string);
        }
        catch (boost::bad_lexical_cast const&)
        {
          ROS_ERROR("Could not convert id >%s< into an integer.", id_string.c_str());
          return false;
        }
      }
      else
      {
        ROS_ERROR("Invalid description string >%s< >%s<. It does not contain a separator.", absolute_filename.c_str(), filename.c_str());
        return false;
      }
      return true;
    }

    /*!
     * @param msg1
     * @param msg2
     * @return True if equal, otherwise False
     */
    bool isEqual(const MessageType& msg1, const MessageType& msg2);
    inline bool isEqual(const double v1, const double v2)
    {
      return (fabs(v1-v2) < 1e-6);
    }
    inline bool isEqual(const std::vector<double> v1, const std::vector<double> v2)
    {
      if( v1.size() != v2.size() )
      {
        return false;
      }
      for (unsigned int i = 0; i < v1.size(); ++i)
      {
        if (!isEqual(v1[i], v2[i]))
        {
          return false;
        }
      }
      return true;
    }

    /*!
     * @param name
     * @param id
     */
    std::string appendId(const std::string& name, const int id)
    {
      std::stringstream ss; ss << id;
      return (name + DESCRIPTION_ID_SEPARATOR + ss.str());
    }

    /*!
     */
    bool initialized_;

    /*!
     */
    std::map<std::string, MessageType> map_;

    /*! Sets the DMP id according to the provided name
     * It also changes the dmp name
     * @param msg
     * @param name
     * @return True on success, otherwise False
     */
    bool add(MessageType& msg, std::string& name);

    /*!
     * @param msg
     * @param name
     * @return True on success, otherwise False
     */
    bool get(MessageType& msg, const std::string& name);

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
    catch (std::exception& ex)
    {
      ROS_ERROR("Library directory >%s< could not be created: %s.", absolute_library_directory_path_.file_string().c_str(), ex.what());
      return false;
    }
    return (initialized_ = reload());
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::reload()
  {
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    std::vector<std::string> filenames;
    for (boost::filesystem::directory_iterator itr(absolute_library_directory_path_); itr != end_itr; ++itr)
    {
      filenames.push_back(itr->path().file_string());
    }
    std::sort(filenames.begin(), filenames.end());
    for (int i = 0; i < (int)filenames.size(); ++i)
    {
      MessageType dmp_message;
      if (!usc_utilities::FileIO<MessageType>::readFromBagFile(dmp_message, DMPType::getVersionString(), filenames[i], false))
      {
        ROS_ERROR("Problems reading >%s<. Cannot reload DMP library from disc.", filenames[i].c_str());
        return false;
      }
      // remove directories, trailing id, and bag file ending
      std::string name;
      int id;
      ROS_VERIFY_MSG(parseName(filenames[i], name, id), "Read DMP >%s< from library that cannot be parsed. This should never happen.", filenames[i].c_str());
      if (!add(dmp_message, name))
      {
        ROS_ERROR("Problems adding >%s<. Cannot reload DMP library from disc.", name.c_str());
        return false;
      }
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::print()
  {
    typename std::map<std::string, MessageType>::iterator it;
    ROS_INFO_COND(map_.empty(), "Libray buffer is empty.");
    ROS_INFO_COND(!map_.empty(), "Libray buffer contains:");
    int index = 1;
    for(it = map_.begin(); it != map_.end(); ++it)
    {
      ROS_INFO("(%i) >%s< has id >%i<.", index, it->first.c_str(), it->second.dmp.parameters.id);
      index++;
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::isEqual(const MessageType& msg1, const MessageType& msg2)
  {
    // dmp parameters
    if( !( isEqual(msg1.dmp.parameters.teaching_duration, msg2.dmp.parameters.teaching_duration)
        && isEqual(msg1.dmp.parameters.execution_duration, msg2.dmp.parameters.execution_duration)
        && isEqual(msg1.dmp.parameters.cutoff, msg2.dmp.parameters.cutoff)
        && (msg1.dmp.parameters.type == msg2.dmp.parameters.type)
      /*  && (msg1.dmp.parameters.id == msg2.dmp.parameters.id) */ ))
      // && (msg1.dmp.state.is_learned == msg2.dmp.state.is_learned)
      // && (msg1.dmp.state.is_setup == msg2.dmp.state.is_setup)
      // && (msg1.dmp.state.is_start_set == msg2.dmp.state.is_start_set)
      // && isEqual(msg1.dmp.state.current_time.delta_t, msg2.dmp.state.current_time.delta_t)
      // && isEqual(msg1.dmp.state.current_time.tau, msg2.dmp.state.current_time.tau)
      // && (msg1.dmp.state.num_training_samples == msg2.dmp.state.num_training_samples)
      // && (msg1.dmp.state.num_generated_samples == msg2.dmp.state.num_generated_samples)
      // && (msg1.dmp.state.seq == msg2.dmp.state.seq) ))
    {
      return false;
    }

    // canonical system parameters
    if ( !isEqual(msg1.canonical_system.canonical_system.parameters.alpha_x, msg2.canonical_system.canonical_system.parameters.alpha_x) )
    {
      return false;
    }

    // transformation system parameters
    if( !(msg1.transformation_systems.size() == msg2.transformation_systems.size() ))
    {
      return false;
    }
    for (unsigned int i = 0; i < msg1.transformation_systems.size(); ++i)
    {
      if (! (msg1.transformation_systems[i].transformation_system.parameters.size() == msg2.transformation_systems[i].transformation_system.parameters.size()))
      {
        return false;
      }
      for (unsigned int j = 0; j < msg1.transformation_systems[i].transformation_system.parameters.size(); ++j)
      {
        if (!(isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].initial_start,
                      msg2.transformation_systems[i].transformation_system.parameters[j].initial_start)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].initial_goal,
                       msg2.transformation_systems[i].transformation_system.parameters[j].initial_goal)
            && (msg1.transformation_systems[i].transformation_system.parameters[j].name.compare(
                msg2.transformation_systems[i].transformation_system.parameters[j].name) == 0)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.num_rfs,
                       msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.num_rfs)
            && (msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.use_offsets
                == msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.use_offsets)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.widths,
                       msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.widths)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.centers,
                       msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.centers)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.slopes,
                       msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.slopes)
            && isEqual(msg1.transformation_systems[i].transformation_system.parameters[j].lwr_model.offsets,
                       msg2.transformation_systems[i].transformation_system.parameters[j].lwr_model.offsets)))
        {
          return false;
        }
      }
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::add(MessageType& msg, std::string& name)
  {
    ROS_DEBUG("Adding DMP with input >%s<.", name.c_str());
    typename std::map<std::string, MessageType>::iterator it;
    int index = 1; // start at one
    bool found = false;
    for (it = map_.begin(); !found && it != map_.end(); ++it)
    {
      // get description of library dmp
      std::string description; int id;
      if(!parseName(it->first, description, id))
      {
        ROS_ERROR("Invalid filename >%s< stored in local memory. This should never happen.", it->first.c_str());
        return false;
      }
      // if description matches...
      if (description.compare(name) == 0)
      {
        // and if the DMPs are the same...
        if (isEqual(it->second, msg))
        {
          ROS_INFO("DMP already contained. Nevertheless, overwriting DMP >%s< and not changing id >%i<.", name.c_str(), it->second.dmp.parameters.id);
          msg.dmp.parameters.id = it->second.dmp.parameters.id;
          it->second = msg;
          name = appendId(name, msg.dmp.parameters.id);
          found = true;
        }
      }
      index++;
    }

    if (!found)
    {
      // ROS_ASSERT_MSG(msg.dmp.parameters.id == 0, "Provided DMP >%s< is not contained in the library but has an assigned ID >%i<. This should never happen.", name.c_str(), msg.dmp.parameters.id);
      ROS_INFO("Adding DMP >%s< and changing id from >%i< to >%i<.", name.c_str(), msg.dmp.parameters.id, index);
      msg.dmp.parameters.id = index;
      name = appendId(name, msg.dmp.parameters.id);
      map_.insert(typename std::pair<std::string, MessageType>(name, msg));
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::get(MessageType& msg, const std::string& name)
  {
    typename std::map<std::string, MessageType>::iterator it = map_.find(name);
    if (it == map_.end())
    {
      return false;
    }
    ROS_INFO("Found DMP >%s< with id >%i<.", name.c_str(), it->second.dmp.parameters.id);
    msg = it->second;
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::addDMP(MessageType& dmp_message,
                                                std::string& name)
  {
    if(name.empty())
    {
      ROS_ERROR("Cannot add DMP without name. Name must be specified.");
      return false;
    }
    // sets id in the msg and appends it to the name
    if(!add(dmp_message, name))
    {
      return false;
    }
    std::string filename = getBagFileName(name);
    ROS_DEBUG("Writing into DMP Library at >%s<.", filename.c_str());
    return dmp::DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(dmp_message, filename, false);
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::getDMP(const std::string& name,
                                                MessageType& dmp_message)
  {
    // check whether it is in the cache.
    if(get(dmp_message, name))
    {
      return true;
    }
    int id;
    std::string description;
    if(!parseName(name, description, id))
    {
      return false;
    }

    std::string filename = getBagFileName(name);
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr(absolute_library_directory_path_); itr != end_itr; ++itr)
    {
      ROS_DEBUG("Checking: >%s< and >%s<.", itr->path().file_string().c_str(), filename.c_str());
      if(filename.compare(itr->path().file_string()) == 0)
      {
        if(!usc_utilities::FileIO<MessageType>::readFromBagFile(dmp_message, DMPType::getVersionString(), filename, false))
        {
          ROS_ERROR("Problems reading >%s<. Cannot return DMP.", filename.c_str());
          return false;
        }
        return true;
      }
    }
    ROS_ERROR("Could not find DMP with name >%s<.", name.c_str());
    return false;
  }
}

#endif /* DMP_LIBRARY_H_ */
