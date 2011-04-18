/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		dmp_library_client.h

  \author	Peter Pastor
  \date		Jan 26, 2011

 *********************************************************************/

#ifndef DMP_LIBRARY_CLIENT_H_
#define DMP_LIBRARY_CLIENT_H_

// system includes
#include <string>

#include <dynamic_movement_primitive/icra2009_dynamic_movement_primitive.h>
#include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes
#include <skill_library/dmp_library.h>

namespace skill_library
{

class DMPLibraryClient
{

public:

  DMPLibraryClient() {};
  virtual ~DMPLibraryClient() {};

  /*!
   * @param data_directory_name
   * @return
   */
  bool initialize(const std::string& library_root_directory);

  /*!
   * @param dmp
   * @param name
   * @return
   */
  bool addDMP(const dmp_lib::DMPPtr& dmp,
              const std::string& name);

  /*! ICRA2009 functions
   */
  bool addDMP(const dmp::ICRA2009DMP::DMPMsg& msg,
              const std::string& name);
  bool getDMP(const std::string& name,
              dmp::ICRA2009DMP::DMPMsg& dmp_message);

  /*! NC2010 functions
   */
  bool addDMP(const dmp::NC2010DMP::DMPMsg& msg,
              const std::string& name);
  bool getDMP(const std::string& name,
              dmp::NC2010DMP::DMPMsg& dmp_message);

private:

  /*!
   */
  DMPLibrary<dmp::ICRA2009DMP, dmp::ICRA2009DMPMsg> icra2009_dmp_library_;
  DMPLibrary<dmp::NC2010DMP, dmp::NC2010DMPMsg> nc2010_dmp_library_;

};

}


#endif /* DMP_LIBRARY_CLIENT_H_ */
