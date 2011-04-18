/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		variable_name_map.h

  \author	Peter Pastor, Mrianl Kalakrishnan
  \date		Feb 27, 2011

 *********************************************************************/

#ifndef VARIABLE_NAME_MAP_H_
#define VARIABLE_NAME_MAP_H_

// system includes
#include <string>
#include <vector>
#include <tr1/unordered_map>

// local includes

namespace pr2_dynamic_movement_primitive_controller
{

class VariableNameMap
{

public:

  /*! Constructor
   */
  VariableNameMap()
    : initialized_(false), start_index_(0) {};
  /*! Destructor
   */
  virtual ~VariableNameMap() {};

  /*!
   * @param supported_variable_names
   * @param used_variable_names
   * @return True on success, otherwise False
   */
  bool initialize(const std::vector<std::string>& supported_variable_names,
                  const std::vector<std::string>& used_variable_names,
                  const int start_index = 0);

  /*!
   * Uses the joint_names provided by SL as supported_variable_names
   * @param supported_variable_names
   * @return True on success, otherwise False
   */
  bool initialize(const std::vector<std::string>& used_variable_names,
                  const int start_index = 0);

  /*!
   * REAL-TIME REQUIREMENTS
   */
  void reset();

  /*!
   * @param used_variable_name
   * @param index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool set(const std::string& used_variable_name, const int index);

  /*!
   * @param used_index
   * @param supported_index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getSupportedVariableIndex(const int used_index, int& supported_index) const;

  /*!
   * @param supported_index
   * @param used_index
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getUsedVariableIndex(const int supported_index, int& used_index) const;

  /**
   * Gets the index to the variable name
   * @param name
   * @return True on success, otherwise False
   * REAL-TIME REQUIREMENTS
   */
  bool getSupportedVariableIndex(const std::string& name, int& index) const;

private:

  bool initialized_;
  int start_index_;

  std::tr1::unordered_map<std::string, int> supported_name_to_index_map_;
  std::vector<int> used_to_supported_map_;
  std::vector<int> supported_to_used_map_;

};

}

#endif /* VARIABLE_NAME_MAP_H_ */
