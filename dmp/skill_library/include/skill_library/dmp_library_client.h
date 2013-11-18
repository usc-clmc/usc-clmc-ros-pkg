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
// #include <dynamic_movement_primitive/nc2010_dynamic_movement_primitive.h>

// local includes
#include <skill_library/dmp_library.h>

namespace skill_library
{

class DMPLibraryClient
{

public:

  /*! Constructor
   */
  DMPLibraryClient() {};
  /*! Destructor
   */
  virtual ~DMPLibraryClient() {};

  /*!
   * @param data_directory_name
   * @return True on success, otherwise False
   */
  bool initialize(const std::string& library_root_directory);

  /*!
   * @param dmp
   * @return True on success, otherwise False
   */
  bool addDMP(dmp_lib::DMPPtr& dmp);

  /*! ICRA2009 functions
   */
  bool addDMP(dmp::ICRA2009DMP::DMPMsg& msg);
  bool getDMP(const std::string& name,
              dmp::ICRA2009DMP::DMPMsg& dmp_message);

  //  /*! NC2010 functions
  //   */
  //  bool addDMP(dmp::NC2010DMP::DMPMsg& msg);
  //  bool getDMP(const std::string& name,
  //              dmp::NC2010DMP::DMPMsg& dmp_message);

  /*! Reloads all DMPs into a buffer
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

private:

  /*!
   */
  DMPLibrary<dmp::ICRA2009DMP, dmp::ICRA2009DMPMsg> icra2009_dmp_library_;
  // DMPLibrary<dmp::NC2010DMP, dmp::NC2010DMPMsg> nc2010_dmp_library_;

};

}


#endif /* DMP_LIBRARY_CLIENT_H_ */
