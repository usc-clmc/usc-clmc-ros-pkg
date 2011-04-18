/*
 * sl_config_file_handler.h
 *
 *  Created on: Dec 14, 2010
 *      Author: righetti
 */

#ifndef SL_CONFIG_FILE_HANDLER_H_
#define SL_CONFIG_FILE_HANDLER_H_

#include <vector>
#include <string>

namespace usc_utilities
{

bool readSLParameterPoolData(std::string filename, std::string keyword, int n_values, std::vector<double>& values);

}

#endif /* SL_CONFIG_FILE_HANDLER_H_ */
