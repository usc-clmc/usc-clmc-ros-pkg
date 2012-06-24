/*********************************************************************
  Computational Learning and Motor Control Lab
  University of Southern California
  Prof. Stefan Schaal 
 *********************************************************************
  \remarks		...
 
  \file		locally_weighted_regression.h

  \author	Peter Pastor
  \date		Nov 4, 2010

 *********************************************************************/

#ifndef LOCALLY_WEIGHTED_REGRESSION_BASE_H_
#define LOCALLY_WEIGHTED_REGRESSION_BASE_H_

// system includes
#include <vector>
#include <string>

#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

// local includes
#include <lwr_lib/lwr_parameters.h>
#include <lwr_lib/status.h>

namespace lwr_lib
{

/*!
 */
class LWR : public Status
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! Constructor
     */
    LWR() {};

    /*! Destructor
     */
    virtual ~LWR() {};

    /*!
     * @param lwr_model
     * @return True if equal, otherwise False
     */
    bool operator==(const LWR& lwr_model) const
    {
      return ((isInitialized() && lwr_model.isInitialized())
          && (*parameters_ == *(lwr_model.parameters_)) );
    }
    bool operator!=(const LWR& lwr_model) const
    {
      return !(*this == lwr_model);
    }

    /*! Assignment operator
     * @param lwr_model
     * @return
     */
    LWR& operator=(const LWR& lwr_model);

    /*!
     * @param parameters
     * @return
     */
    bool initialize(const LWRParamPtr parameters);

    /*!
     * @param x_input_vector
     * @param y_target_vector
     * @return True on success, otherwise False
     */
    bool learn(const Eigen::VectorXd& x_input_vector, const Eigen::VectorXd& y_target_vector);

    /*!
     * @param x_query
     * @param y_prediction
     * @return True on success, otherwise False
     * REAL-TIME REQUIREMENTS
     */
    bool predict(const double x_query, double& y_prediction);

    /*! Gets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool getThetas(Eigen::VectorXd& thetas) const;

    /*! Gets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool getThetas(std::vector<double>& thetas) const;

    /*! Sets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool setThetas(const Eigen::VectorXd& thetas);

    /*! Sets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool setThetas(const std::vector<double>& thetas);

    /*! Updates the theta vectors (thetas += delta_thetas)
     * @param delta_thetas
     * @return True on success, false on failure
     */
    bool updateThetas(const Eigen::VectorXd& delta_thetas);

    /*! Sets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool setWidthsAndCenters(const Eigen::VectorXd& widths, const Eigen::VectorXd& centers);

    /*! Sets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool setWidthsAndCenters(const std::vector<double>& widths, const std::vector<double>& centers);

    /*! Gets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(Eigen::VectorXd& widths, Eigen::VectorXd& centers) const;

    /*! Gets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(std::vector<double>& widths, std::vector<double>& centers) const;

    /*! Sets the offset vector
     * @param offsets
     * @return True on success, false on failure
     */
    bool setOffsets(const Eigen::VectorXd& offsets);

    /*! Sets the offset vector
     * @param offsets
     * @return True on success, false on failure
     */
    bool setOffsets(const std::vector<double>& offsets);

    /*! Gets the offset vector
     * @param offsets
     * @return True on success, false on failure
     */
    bool getOffsets(Eigen::VectorXd& offsets) const;

    /*! Gets the offset vector
     * @param offsets
     * @return True on success, false on failure
     */
    bool getOffsets(std::vector<double>& offsets) const;

    /*! Sets the number of receptive fields
     *
     * @param num_rfs
     */
    bool setNumRFS(const int num_rfs);

    /*! Returns the number of receptive fields
     * @return
     */
    int getNumRFS() const;

//    /*!
//     * @return
//     */
//    LWRParamPtr getParameters() const;
//
//    /*!
//     * @param lwr_parameters
//     * @return True on success, otherwise False
//     */
//    bool getParameters(LWRParameters& lwr_parameters) const;

    /*!
     * @param x_input_vector
     * @param basis_function_matrix
     * @return True on success, false on failure
     */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& x_input_vector, Eigen::MatrixXd& basis_function_matrix) const;

    /*!
     * @param x_input
     * @param basis_functions
     * @return True on success, false on failure
     */
    bool generateBasisFunctionVector(const double x_input, Eigen::VectorXd& basis_functions) const;

    /*!
     * @param x_input
     * @param rfs_index
     * @param basis_functions
     * @return True on success, false on failure
     */
    bool generateBasisFunction(const double x_input, const int rfs_index, double& basis_function) const;

    /*! Return a string containing information about the LWR model
     * @return string containing information about the LWR model
     */
    std::string getInfoString() const;

private:

    /*! Parameters of the LWR model
     */
    LWRParamPtr parameters_;

    /*! Evaluates the kernel
     * REAL-TIME REQUIREMENTS
     */
    double evaluateKernel(const double x_input, const int center_index) const;

};

/*! Abbreviation for convinience
 */
typedef boost::shared_ptr<LWR> LWRPtr;
typedef boost::shared_ptr<LWR const> LWRConstPtr;

}

#endif /* LOCALLY_WEIGHTED_REGRESSION_BASE_H_ */
