/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks       ...

 \file      lwr_test.cpp

 \author    Peter Pastor
 \date      Dec 6, 2010

 *********************************************************************/

/** \author Peter Pastor */

// system includes
#include <time.h>
#include <iostream>
#include <fstream>

// ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <usc_utilities/param_server.h>
#include <usc_utilities/assert.h>
#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>

//local includes
#include <locally_weighted_regression/locally_weighted_regression.h>

using namespace Eigen;
using namespace lwr;

class LocallyWeightedRegressionTest
{

public:

    /*! Constructor
     */
    LocallyWeightedRegressionTest(){};
    /*! Destructor
     */
    virtual ~LocallyWeightedRegressionTest() {};

    /*!
     * @return
     */
    bool initialize();

    /*!
     * @return
     */
    bool runLearnTest();
    bool runWriteToDiscTest();
    bool runBasisFunctionMatrixTest();
    // bool runTestAssignmentOperator();

private:

    LWRPtr lwr_;

    std::string data_directory_name_;
    std::string library_directory_name_;
    std::string bag_file_name_;

    double activation_;
    int num_rfs_;
    bool exponentially_spaced_;
    bool use_offset_;

    double cutoff_;

    int num_data_learn_;
    int num_data_query_;
    double mse_prediction_error_threshold_;
    double mse_rescaling_error_threshold_;

    VectorXd test_x_;
    VectorXd test_y_;

    double targetFunction(const double test_x);

};

double LocallyWeightedRegressionTest::targetFunction(const double test_x)
{
	// return -pow(test_x - 0.5, 2);
  return pow(test_x - 0.5, 2);
}

bool LocallyWeightedRegressionTest::initialize()
{
    ros::NodeHandle node_handle(std::string("/LocallyWeightedRegressionTest/test_parameters"));

    ros::NodeHandle dmp_node_handle(node_handle, std::string("dmp"));
    ROS_VERIFY(usc_utilities::read(dmp_node_handle, std::string("cutoff"), cutoff_));

    lwr_.reset(new LocallyWeightedRegression());
    ROS_VERIFY(lwr_->initFromNodeHandle(node_handle, cutoff_));

    // get package name and then get the package path
    std::string package_name;
    ROS_VERIFY(usc_utilities::read(node_handle, std::string("package_name"), package_name));

    // TODO: fix this
    // std::string package_path = ros::package::getPath(package_name);
    std::string package_path = "/home/arm_user/ARM/usc-clmc-ros-pkg/dmp/locally_weighted_regression";
    usc_utilities::appendTrailingSlash(package_path);

    std::string sub_directory_name;
    ROS_VERIFY(usc_utilities::read(node_handle, std::string("library_directory_name"), sub_directory_name));

    library_directory_name_.assign(package_path + sub_directory_name);
    usc_utilities::appendTrailingSlash(library_directory_name_);
    ROS_INFO("Library directory is set to %s.", library_directory_name_.c_str());

    sub_directory_name.clear();
    ROS_VERIFY(usc_utilities::read(node_handle, std::string("data_directory_name"), sub_directory_name));
    data_directory_name_.assign(package_path + sub_directory_name);
    usc_utilities::appendTrailingSlash(data_directory_name_);
    ROS_INFO("Data directory is set to %s.", data_directory_name_.c_str());

    // create directory if it doesn't exist:
    boost::filesystem::create_directories(library_directory_name_);
    boost::filesystem::create_directories(data_directory_name_);

    ROS_VERIFY(usc_utilities::read(node_handle, std::string("bag_file_name"), bag_file_name_));

    ROS_VERIFY(usc_utilities::read(node_handle, std::string("num_data_learn"), num_data_learn_));
    ROS_VERIFY(usc_utilities::read(node_handle, std::string("num_data_query"), num_data_query_));

    ROS_VERIFY(usc_utilities::read(node_handle, std::string("mse_prediction_error_threshold"), mse_prediction_error_threshold_));
    ROS_VERIFY(usc_utilities::read(node_handle, std::string("mse_rescaling_error_threshold"), mse_rescaling_error_threshold_));

    ros::NodeHandle lwr_node_handle(node_handle, std::string("lwr"));
    ROS_VERIFY(usc_utilities::read(lwr_node_handle, std::string("activation"), activation_));
    ROS_VERIFY(usc_utilities::read(lwr_node_handle, std::string("num_rfs"), num_rfs_));
    ROS_VERIFY(usc_utilities::read(lwr_node_handle, std::string("use_offset"), use_offset_));
    ROS_VERIFY(usc_utilities::read(lwr_node_handle, std::string("exponentially_spaced"), exponentially_spaced_));

    // generate input vector
    test_x_ = VectorXd::Zero(num_data_learn_);
    test_x_(0) = 0;
    double dx = static_cast<double> (1.0) / (test_x_.size() - 1);
    for (int i = 1; i < test_x_.size(); i++)
    {
        test_x_(i) = test_x_(i - 1) + dx;
    }

    // generate target
    test_y_ = VectorXd::Zero(test_x_.size());
    for (int i = 1; i < test_x_.size(); i++)
    {
        test_y_(i) = targetFunction(test_x_(i));
    }

    return true;
}

bool LocallyWeightedRegressionTest::runLearnTest()
{
    // initialize random seed
    srand(time(NULL));

    // learn parameters
    if (!lwr_->learn(test_x_, test_y_))
    {
        ROS_ERROR("Could not learn weights.");
        return false;
    }

    VectorXd test_xq = VectorXd::Zero(num_data_query_);
    test_xq(0) = 0;
    double dx = static_cast<double> (1.0) / (test_xq.size() - 1);
    for (int i = 1; i < test_xq.size(); i++)
    {
        test_xq(i) = test_xq(i - 1) + dx;
    }

    // get predictions
    VectorXd test_yp = VectorXd::Zero(test_xq.size());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_->predict(test_xq(i), test_yp(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
    }

    // log data
    std::ofstream outfile;
    outfile.open(std::string(data_directory_name_ + std::string("test_x.txt")).c_str());
    outfile << test_x_;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_y.txt")).c_str());
    outfile << test_y_;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_xq.txt")).c_str());
    outfile << test_xq;
    outfile.close();

    outfile.open(std::string(data_directory_name_ + std::string("test_yp.txt")).c_str());
    outfile << test_yp;
    outfile.close();

    // compute mean squared error
    double mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(targetFunction(test_xq(i)) - test_yp(i), 2);
    }
    if (mse > mse_prediction_error_threshold_)
    {
        ROS_ERROR("ERROR: MSE of the prediciton (%f) is larger than the threshold (%f).", mse, mse_prediction_error_threshold_);
        return false;
    }
    return true;
}

bool LocallyWeightedRegressionTest::runBasisFunctionMatrixTest()
{
    MatrixXd basis_function_matrix;
    basis_function_matrix = MatrixXd::Zero(test_x_.size(), num_rfs_);
    if (!lwr_->generateBasisFunctionMatrix(test_x_, basis_function_matrix))
    {
        ROS_ERROR("Could not get basis function matrix");
        return false;
    }
    // log data
    std::ofstream outfile;
    outfile.open(std::string(data_directory_name_ + std::string("basis_function_matrix.txt")).c_str());
    outfile << basis_function_matrix;
    outfile.close();

    return true;
}

bool LocallyWeightedRegressionTest::runWriteToDiscTest()
{

    // generate query
    VectorXd test_xq = VectorXd::Zero(num_data_query_);
    test_xq(0) = 0;
    double dx = static_cast<double> (1.0) / (test_xq.size() - 1);
    for (int i = 1; i < test_xq.size(); i++)
    {
        test_xq(i) = test_xq(i - 1) + dx;
    }

    // learn parameters
    if (!lwr_->learn(test_x_, test_y_))
    {
        ROS_ERROR("Could not learn weights.");
        return false;
    }

    // write LWR model to disc
    if (!lwr_->writeToDisc(library_directory_name_ + bag_file_name_))
    {
        ROS_ERROR("Could not write LWR model to file.");
        return false;
    }

    // create new LWR model and read previous one from disc
    LWRPtr lwr_copy;
    lwr_copy.reset(new LocallyWeightedRegression());
    ROS_VERIFY(lwr_copy->readFromDisc(library_directory_name_ + bag_file_name_));

    if(*lwr_copy != *lwr_)
    {
      ROS_ERROR("LWR model and its copy read from file are not the same.");
      return false;
    }

    // get predictions
    VectorXd test_yp = VectorXd::Zero(test_xq.size());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_->predict(test_xq(i), test_yp(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
    }

    ROS_INFO_STREAM(lwr_copy->getInfoString());

    // get predictions
    VectorXd test_yp_copy = VectorXd::Zero(test_xq.rows(), test_xq.cols());
    for (int i = 0; i < test_xq.size(); i++)
    {
        if (!lwr_copy->predict(test_xq(i), test_yp_copy(i)))
        {
            ROS_ERROR("Could not predict from LWR model.");
            return false;
        }
    }

    // log data
    std::ofstream outfile;
    outfile.open(std::string(data_directory_name_ + std::string("test_yp_copy.txt")).c_str());
    outfile << test_yp_copy;
    outfile.close();

    // compute mean squared error
    double mse = 0;
    for (int i = 0; i < test_xq.size(); i++)
    {
        mse += pow(test_yp(i) - test_yp_copy(i), 2);
    }
    if (mse > 0)
    {
        ROS_ERROR("MSE between original and copy is %f and therefore not equals zero.", mse);
        return false;
    }

    return true;
}

//bool LocallyWeightedRegressionTest::runTestAssignmentOperator()
//{
//
//    if (!initialize())
//    {
//        return false;
//    }
//
//    // test assignemnt operator
//    LWRPtr lwr_copy;
//    lwr_copy.reset(new LocallyWeightedRegression());
//    lwr_another_copy = lwr_copy;
//    VectorXd test_yp_another_copy = VectorXd::Zero(test_xq.size());
//    for (int i = 0; i < test_xq.size(); i++)
//    {
//        if (!lwr_another_copy.predict(test_xq(i), test_yp_another_copy(i)))
//        {
//            ROS_ERROR("Could not predict from the second copy of LWR model.");
//            return false;
//        }
//    }
//    // compute mean squared error
//    mse = 0;
//    for (int i = 0; i < test_xq.size(); i++)
//    {
//        mse += pow(test_yp(i) - test_yp_another_copy(i), 2);
//    }
//    if (mse > mse_rescaling_error_threshold_)
//    {
//        ROS_ERROR("MSE of the prediciton (%f) is larger than the threshold (%f).", mse, mse_rescaling_error_threshold_);
//        return false;
//    }
//
//    // checking whether we made a deep copy and not just using the same object
//    VectorXd thetas = VectorXd::Zero(num_rfs_);
//    if (!lwr_another_copy.getThetas(thetas))
//    {
//        ROS_ERROR("Could not obtain thetas.");
//        return false;
//    }
//    for (int i = 0; i < num_rfs_; i++)
//    {
//        thetas(i) = i;
//    }
//    if (!lwr_another_copy.setThetas(thetas))
//    {
//        ROS_ERROR("Could not set thetas.");
//        return false;
//    }
//
//    test_yp_copy.setZero();
//    test_yp_another_copy.setZero();
//    for (int i = 0; i < test_xq.size(); i++)
//    {
//        if (!lwr_copy.predict(test_xq(i), test_yp_copy(i)))
//        {
//            ROS_ERROR("Could not predict from LWR model.");
//            return false;
//        }
//        if (!lwr_another_copy.predict(test_xq(i), test_yp_another_copy(i)))
//        {
//            ROS_ERROR("Could not predict from the second copy of LWR model.");
//            return false;
//        }
//    }
//
//    mse = 0;
//    for (int i = 0; i < test_xq.size(); i++)
//    {
//        mse += pow(test_yp_copy(i) - test_yp_another_copy(i), 2);
//    }
//    if (mse < mse_rescaling_error_threshold_)
//    {
//        ROS_ERROR("MSE of the prediciton (%f) is less than the threshold (%f) --> Copy constructor does not make a deep copy.", mse, mse_rescaling_error_threshold_);
//        return false;
//    }
//
//    return true;
//}

TEST(lwr_tests, run_lwr_learn_test)
{
    LocallyWeightedRegressionTest lwr_test;
    EXPECT_TRUE(lwr_test.initialize());
    EXPECT_TRUE(lwr_test.runLearnTest());
}
TEST(lwr_tests, run_lwr_write_to_disc_test)
{
    LocallyWeightedRegressionTest lwr_test;
    EXPECT_TRUE(lwr_test.initialize());
    EXPECT_TRUE(lwr_test.runWriteToDiscTest());
}
//TEST(lwr_tests, run_lwr_basis_function_test)
//{
//    LocallyWeightedRegressionTest lwr_test;
//    EXPECT_TRUE(lwr_test.initialize());
//    EXPECT_TRUE(lwr_test.runBasisFunctionMatrixTest());
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lwr_tests");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
