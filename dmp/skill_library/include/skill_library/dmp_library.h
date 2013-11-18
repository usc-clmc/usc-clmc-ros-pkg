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
#include <boost/format.hpp>
#include <boost/functional/hash.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <usc_utilities/assert.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/file_io.h>

#include <geometry_msgs/Pose.h>

#include <dynamic_movement_primitive/dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/dynamic_movement_primitive_io.h>

// local includes
#include <skill_library/dmp_library_io.h>

namespace skill_library
{

static const std::string SLASH = "/";
static const std::string DESCRIPTION_ID_SEPARATOR = "_";
static const std::string ID_STRING_SEPARATOR = "-";
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

    /*! Retrieves the DMP from the library
     * @param description must be of the form <description_id>
     * @param dmp_message
     * @return
     */
    bool getDMP(const std::string& description,
                MessageType& dmp_message);

    /*! Adds DMP to library
     * Note: the dmp.parameters.description must be of the form <description_id>
     * @param dmp_message
     * @return True on success, otherwise False
     */
    bool addDMP(MessageType& dmp_message);

    /*! Reloads all DMPs from disc into a buffer
     * @return True on success, otherwise False
     */
    bool reload();

    /*!
     * @return True on success, otherwise False
     */
    bool print();

    /*!
     * @param description
     * @return True on success, otherwise False
     */
    bool printInfo(const std::string& description);

    /*!
     * @param name
     * @return converted absolute bag file name
     */
    std::string getBagFileName(const std::string& name)
    {
      return absolute_library_directory_path_.file_string() + SLASH + name + BAG_FILE_ENDING;
    }

  private:

    std::string getIdString(const MessageType& dmp_message) const;

    std::string formatVariable(const double& variable) const;
    std::string formatVariables(const std::vector<double>& variable) const;
    std::string formatVariables(const geometry_msgs::Pose& pose) const;

    std::string removeBagFileEnding(const std::string& filename)
    {
      std::string name = filename;
      size_t bag_separater_pos = name.rfind(BAG_FILE_ENDING);
      if (bag_separater_pos != std::string::npos && name.length() >= BAG_FILE_ENDING.length())
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

    /*! This contains the mapping between DMP (description) names
     */
    std::map<std::string, MessageType> map_;

    /*! Adds DMP to the library
     * @param dmp_message
     * @return True on success, otherwise False
     */
    bool add(MessageType& dmp_message);

    /*!
     * @param dmp_message
     * @param description
     * @return True on success, otherwise False
     */
    bool get(MessageType& dmp_message, const std::string& description);

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
    ROS_INFO("Clearing local buffer.");
    map_.clear();

    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    std::vector<std::string> filenames;
    for (boost::filesystem::directory_iterator itr(absolute_library_directory_path_); itr != end_itr; ++itr)
    {
      filenames.push_back(itr->path().file_string());
    }
    std::sort(filenames.begin(), filenames.end());
    for (unsigned int i = 0; i < filenames.size(); ++i)
    {
      MessageType dmp_message;
      if (!usc_utilities::FileIO<MessageType>::readFromBagFile(dmp_message, DMPType::getVersionString(), filenames[i], false))
      {
        ROS_ERROR("Problems reading >%s<. Cannot reload DMP library from disc.", filenames[i].c_str());
        return false;
      }
      ROS_INFO("Reloading DMP >%s<.", dmp_message.dmp.parameters.description.c_str());
      map_.insert(typename std::pair<std::string, MessageType>(dmp_message.dmp.parameters.description, dmp_message));
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::print()
  {
    typename std::map<std::string, MessageType>::iterator it;
    ROS_WARN_COND(map_.empty(), "Library buffer is empty.");
    ROS_INFO_COND(!map_.empty(), "Library buffer contains:");
    unsigned int index = 0;
    for (it = map_.begin(); it != map_.end(); ++it)
    {
      if (it->first != it->second.dmp.parameters.description)
      {
        ROS_ERROR("Skill library is corrupt. Need to reload.");
        return false;
      }
      ROS_INFO("(%i) >%s<", index++, it->first.c_str());
    }
    return true;
  }

template<class DMPType, class MessageType>
  std::string DMPLibrary<DMPType, MessageType>::getIdString(const MessageType& dmp_message) const
{
  std::string id_string = dmp_message.dmp.parameters.description;
  std::string object_name = dmp_message.dmp.task.object_name;
  if (object_name.empty())
    object_name = "no_object";
  id_string.append(ID_STRING_SEPARATOR + object_name);
  std::string endeffector = "no_endeffector";
  if (dmp_message.dmp.task.endeffector_id == 1)
    endeffector = "right_hand";
  else if (dmp_message.dmp.task.endeffector_id == 2)
    endeffector = "left_hand";
  else if (dmp_message.dmp.task.endeffector_id == 3)
    endeffector = "both_hands";
  id_string.append(ID_STRING_SEPARATOR + endeffector);
  return id_string;
}

template<class DMPType, class MessageType>
  std::string DMPLibrary<DMPType, MessageType>::formatVariable(const double& variable) const
{
  std::stringstream ss;
  ss << boost::format("%5.2f") % variable;
  return ss.str();
}

template<class DMPType, class MessageType>
  std::string DMPLibrary<DMPType, MessageType>::formatVariables(const std::vector<double>& variables) const
{
  std::stringstream ss;
  for (size_t i=0; i<variables.size(); ++i)
  {
    ss << boost::format("% 5.2f ") % variables[i];
  }
  return ss.str();
}

template<class DMPType, class MessageType>
  std::string DMPLibrary<DMPType, MessageType>::formatVariables(const geometry_msgs::Pose& pose) const
{
  std::stringstream ss;
  ss << boost::format("% 5.2f ") % pose.position.x;
  ss << boost::format("% 5.2f ") % pose.position.y;
  ss << boost::format("% 5.2f ") % pose.position.z;
  ss << boost::format("% 5.2f ") % pose.orientation.w;
  ss << boost::format("% 5.2f ") % pose.orientation.x;
  ss << boost::format("% 5.2f ") % pose.orientation.y;
  ss << boost::format("% 5.2f ") % pose.orientation.z;
  return ss.str();
}

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::printInfo(const std::string& description)
  {
    typename std::map<std::string, MessageType>::iterator it;
    ROS_WARN_COND(map_.empty(), "Library buffer is empty.");

    bool found = false;
    for (it = map_.begin(); it != map_.end(); ++it)
    {
      if (it->first.length() >= description.length() && it->first.substr(0, description.length()) == description)
      {
        found = true;
        std::vector<std::string> title;
        std::vector<std::string> dmp_info;
        std::vector<std::string> info1;
        std::vector<std::string> info2;
        std::vector<std::string> names;
        unsigned int num_transformation_systems = it->second.transformation_systems.size();
        title.push_back("DMP: >" + description + "< has " + boost::lexical_cast<std::string>(num_transformation_systems) + " transformation systems.");
        std::string endeffector = "undefined";
        if (it->second.dmp.task.endeffector_id == 1)
          endeffector = "right hand";
        else if (it->second.dmp.task.endeffector_id == 2)
          endeffector = "left hand";
        else if (it->second.dmp.task.endeffector_id == 3)
          endeffector = "both hands";
        dmp_info.push_back("Endeffector:      " + endeffector);
        dmp_info.push_back("Object name:      " + it->second.dmp.task.object_name);
        dmp_info.push_back("Palm to tool:     [" + formatVariables(it->second.dmp.task.palm_to_tool) + "]");
        dmp_info.push_back("Object to tool:   [" + formatVariables(it->second.dmp.task.object_to_tool) + "]");
        dmp_info.push_back("Initial duration:" + formatVariable(it->second.dmp.parameters.teaching_duration) + " sec ");
        unsigned int n = 0;
        for (unsigned int i = 0; i < num_transformation_systems; ++i)
        {
          if (it->second.transformation_systems[i].parameters.size() == 1)
          {
            std::string id = boost::lexical_cast<std::string>(n);
            if (n < 10)
              id += " ";
            id = "[" + id + "]";
            names.push_back(it->second.transformation_systems[i].transformation_system.parameters[0].name);
            info1.push_back(id + " - (" + boost::lexical_cast<std::string>(i) + ") " + it->second.transformation_systems[i].transformation_system.parameters[0].name);
            info2.push_back("initial_start = " + formatVariable(it->second.transformation_systems[i].transformation_system.parameters[0].initial_start)
                              + " initial_goal = " +  formatVariable(it->second.transformation_systems[i].transformation_system.parameters[0].initial_goal));
            n++;
          }
          else
          {
            for (unsigned j = 0; j < it->second.transformation_systems[i].parameters.size(); ++j)
            {
              std::string id = boost::lexical_cast<std::string>(n);
              if (n < 10)
                id += " ";
              id = "[" + id + "]";
              names.push_back(it->second.transformation_systems[i].transformation_system.parameters[j].name);
              info1.push_back(id + " - (" + boost::lexical_cast<std::string>(i) + "/" + boost::lexical_cast<std::string>(j) + ") " + it->second.transformation_systems[i].transformation_system.parameters[j].name);
              info2.push_back("initial_start = " + formatVariable(it->second.transformation_systems[i].transformation_system.parameters[j].initial_start)
                                + " initial_goal = " +  formatVariable(it->second.transformation_systems[i].transformation_system.parameters[j].initial_goal));
              n++;
            }
          }
        }
        for (unsigned int i = 0; i < names.size(); ++i)
        {
          unsigned int variable_name_length = names[i].length();
          ROS_WARN_COND(variable_name_length > 20, "Variable >%s< has more than 20 characters which can become a problem.", names[i].c_str());
        }

        unsigned int max_length = 0;
        for (unsigned int i = 0; i < info1.size(); ++i)
        {
          unsigned int variable_name_length = info1[i].length();
          if (variable_name_length > max_length)
            max_length = variable_name_length;
        }

        for (unsigned int i = 0; i < info1.size(); ++i)
        {
          int extra = max_length - info1[i].length();
          info1[i] += std::string(extra, ' ');
          info1[i] += std::string(" | ");
          info1[i] += info2[i];
        }

        // printing

        ROS_INFO_STREAM("");
        for (unsigned int i = 0; i < title.size(); ++i)
        {
          ROS_WARN_STREAM(title[i]);
        }
        for (unsigned int i = 0; i < dmp_info.size(); ++i)
        {
          ROS_INFO_STREAM(dmp_info[i]);
        }
        for (unsigned int i = 0; i < info1.size(); ++i)
        {
          ROS_INFO_STREAM(info1[i]);
        }
      }
    }

    if (!found)
    {
      ROS_WARN("Description >%s< not contained in library. Maybe need to reload the library ?", description.c_str());
      return false;
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
        && (msg1.dmp.parameters.description == msg2.dmp.parameters.description)))
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
      if ( !(msg1.transformation_systems[i].transformation_system.parameters.size() == msg2.transformation_systems[i].transformation_system.parameters.size()))
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
  bool DMPLibrary<DMPType, MessageType>::add(MessageType& dmp_message)
  {
    std::string description = "";
    int id = 0;
    // error checking
    if (!parseName(dmp_message.dmp.parameters.description, description, id))
    {
      ROS_ERROR("Could not parse name >%s<. Need to be of the form <description_id>. Not adding DMP.", dmp_message.dmp.parameters.description.c_str());
      return false;
    }
    description = getIdString(dmp_message);
    boost::hash<std::string> string_hash;
    dmp_message.dmp.parameters.id = string_hash(dmp_message.dmp.parameters.description);
    typename std::map<std::string, MessageType>::iterator it;
    bool found = false;
    for (it = map_.begin(); !found && it != map_.end(); ++it)
    {
      // if description matches...
      if (description.compare(it->first) == 0)
      {
        ROS_INFO("DMP >%s< already contained. Nevertheless, overwriting it.", it->first.c_str());
        it->second = dmp_message;
        found = true;
      }
    }
    if (!found)
    {
      ROS_INFO("Adding DMP >%s< as >%s< with hash >%i<.", dmp_message.dmp.parameters.description.c_str(), description.c_str(), dmp_message.dmp.parameters.id);
      map_.insert(typename std::pair<std::string, MessageType>(description, dmp_message));
    }
    return true;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::get(MessageType& dmp_message, const std::string& description)
  {
    typename std::map<std::string, MessageType>::iterator it;
    bool found = false;
    std::string id_string = getIdString(dmp_message);
    for (it = map_.begin(); !found && it != map_.end(); ++it)
    {
      if (it->first.length() >= description.length() && it->first.substr(0, description.length()) == description)
      {
        dmp_message = it->second;
        ROS_INFO("Found DMP >%s< as >%s<.", description.c_str(), it->first.c_str());
        found = true;
      }
    }
    return found;
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::addDMP(MessageType& dmp_message)
  {
    // sets id in the msg and appends it to the name
    if (!add(dmp_message))
    {
      return false;
    }
    if (!boost::filesystem::exists(absolute_library_directory_path_))
    {
      try
      {
        boost::filesystem::create_directories(absolute_library_directory_path_);
      }
      catch (std::exception& ex)
      {
        ROS_ERROR_STREAM("Could not create directory >" << absolute_library_directory_path_.filename() << "< : " << std::strerror(errno));
        return false;
      }
    }
    std::string filename = getBagFileName(dmp_message.dmp.parameters.description);
    ROS_DEBUG("Writing into DMP Library at >%s<.", filename.c_str());
    return dmp::DynamicMovementPrimitiveIO<DMPType, MessageType>::writeToDisc(dmp_message, filename, false);
  }

template<class DMPType, class MessageType>
  bool DMPLibrary<DMPType, MessageType>::getDMP(const std::string& description,
                                                MessageType& dmp_message)
  {
    // error checking
    std::string input_description = "";
    int input_id = 0;
    if (!parseName(description, input_description, input_id))
    {
      ROS_ERROR("Could not parse name >%s<. Cannot get DMP.", description.c_str());
      return false;
    }
    // check whether it is in the cache.
    if (get(dmp_message, description))
    {
      return true;
    }
    std::string filename = getBagFileName(description);
    ROS_INFO("DMP description >%s< is not in local cache. Reading it from >%s< instead.",
             description.c_str(), filename.c_str());
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr(absolute_library_directory_path_); itr != end_itr; ++itr)
    {
      ROS_DEBUG("Checking: >%s< and >%s<.", itr->path().file_string().c_str(), filename.c_str());
      if (filename.compare(itr->path().file_string()) == 0)
      {
        if (!usc_utilities::FileIO<MessageType>::readFromBagFile(dmp_message, DMPType::getVersionString(), filename, false))
        {
          ROS_ERROR("Problems reading >%s<. Cannot return DMP.", filename.c_str());
          return false;
        }
        return true;
      }
    }
    ROS_ERROR("Could not find DMP with name >%s<.", description.c_str());
    return false;
  }
}

#endif /* DMP_LIBRARY_H_ */
