/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Mrinal Kalakrishnan */

#include <gtest/gtest.h>
#include <usc_utilities/param_server.h>

using namespace usc_utilities;

TEST(UscUtilitiesParamServer, readDoubleArray)
{
    ros::NodeHandle node_handle("~");
    std::vector<double> array;

    EXPECT_TRUE(readDoubleArray(node_handle, "double_array_good1", array));
    EXPECT_EQ(5, int(array.size()));
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array[i]);
    }

    EXPECT_TRUE(readDoubleArray(node_handle, "double_array_good2", array));
    EXPECT_EQ(5, int(array.size()));
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array[i]);
    }

    EXPECT_TRUE(readDoubleArray(node_handle, "double_array_good3", array));
    EXPECT_EQ(5, int(array.size()));
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array[i]);
    }

    EXPECT_FALSE(readDoubleArray(node_handle, "double_array_bad1", array));
    EXPECT_FALSE(readDoubleArray(node_handle, "double_array_bad2", array));
}

TEST(UscUtilitiesParamServer, readEigenVector)
{
    ros::NodeHandle node_handle("~");
    Eigen::VectorXd array;

    EXPECT_TRUE(readEigenVector(node_handle, "double_array_good1", array));
    EXPECT_EQ(5, array.size());
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array(i));
    }

    EXPECT_TRUE(readEigenVector(node_handle, "double_array_good2", array));
    EXPECT_EQ(5, array.size());
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array(i));
    }

    EXPECT_TRUE(readEigenVector(node_handle, "double_array_good3", array));
    EXPECT_EQ(5, array.size());
    for (int i=0; i<5; ++i)
    {
        EXPECT_DOUBLE_EQ(i+1, array(i));
    }

    EXPECT_FALSE(readEigenVector(node_handle, "double_array_bad1", array));
    EXPECT_FALSE(readEigenVector(node_handle, "double_array_bad2", array));
}

TEST(UscUtilitiesParamServer, readStringArraySpaceSeparated)
{
    ros::NodeHandle node_handle("~");
    std::vector<std::string> array;

    EXPECT_TRUE(readStringArraySpaceSeparated(node_handle, "string_array", array));
    for (int i=0; i<3; ++i)
    {
        char x[2];
        x[0] = 'a' + i;
        x[1] = 0;
        std::string x_str(x);
        EXPECT_EQ(x_str, array[i]);
    }
}
