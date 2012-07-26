/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal 
 *********************************************************************
 \remarks		...
 
 \file		locally_weighted_regression.h

 \author	Peter Pastor
 \date		Dec 6, 2010

 *********************************************************************/

#ifndef LOCALLY_WEIGHTED_REGRESSION_H_
#define LOCALLY_WEIGHTED_REGRESSION_H_

// system includes
#include <ros/ros.h>

#include <lwr_lib/lwr.h>

// local includes
#include <locally_weighted_regression/Model.h>

namespace lwr
{

/*! Abbreviation for convinience
 */
typedef locally_weighted_regression::Model LWRModelMsg;

/*!
 */
class LocallyWeightedRegression
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! Constructor
     */
    LocallyWeightedRegression() :
      initialized_(false) {};

    /*! Destructor
     */
    virtual ~LocallyWeightedRegression() {};

    /*!
     * @param lwr
     * @return True if equal, otherwise False
     */
    bool operator==(const LocallyWeightedRegression lwr) const
    {
      return (initialized_ && lwr.initialized_ && (*lwr_model_ == *(lwr.lwr_model_)) );
    }
    bool operator!=(const LocallyWeightedRegression lwr) const
    {
      return !(*this == lwr);
    }

    /*! Initializes LWR model from message
     * @param model
     * @param cutoff This parameter is only used when basis function are exponentially spaced. Thus, when
     * basis function are exponentially spaced, this value needs to be set to a possitive value (e.g. 0.001).
     * @return True on success, otherwise False
     */
    bool initFromNodeHandle(ros::NodeHandle& node_handle,
                            const double cutoff = -1);

    /*! Initializes LWR model from message
     * @param model
     * @return True on success, otherwise False
     */
    bool initFromMessage(const LWRModelMsg& model);

    /*!
     * @return
     */
    bool isInitialized() const;

    /*!
     * @param x_input_vector
     * @param y_target_vector
     * @return
     */
    bool learn(const Eigen::VectorXd& x_input_vector,
               const Eigen::VectorXd& y_target_vector);

    /*!
     * @param x_query
     * @param y_prediction
     * @return True on success, otherwise False
     * REAL-TIME REQUIREMENTS
     */
    bool predict(const double x_query,
                 double& y_prediction);

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
    bool setWidthsAndCenters(const Eigen::VectorXd& widths,
                             const Eigen::VectorXd& centers);

    /*! Sets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool setWidthsAndCenters(const std::vector<double>& widths,
                             const std::vector<double>& centers);

    /*! Gets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(Eigen::VectorXd& widths,
                             Eigen::VectorXd& centers) const;

    /*! Gets width and centers of local models
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(std::vector<double>& widths,
                             std::vector<double>& centers) const;

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

    /*!
     * @param x_input_vector
     * @param basis_function_matrix
     * @return True on success, false on failure
     */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& x_input_vector,
                                     Eigen::MatrixXd& basis_function_matrix);

    /*! Return a string containing information about the LWR model
     * @return string containing information about the LWR model
     */
    std::string getInfoString() const;

    /*! Writes LWR model to message
     * @param model
     * @return True on success, otherwise False
     */
    bool writeToMessage(LWRModelMsg& model);

    /*! Reads LWR model from bag file
     * @param bag_file_name
     * @return True on success, otherwise False
     */
    bool readFromDisc(const std::string& bag_file_name);

    /*! Writes LWR model to bag file
     * @param bag_file_name
     * @return True on success, otherwise False
     */
    bool writeToDisc(const std::string& bag_file_name);

    /*!
     * @return
     */
    lwr_lib::LWRPtr getModel() const;

private:

    /*!
     */
    bool initialized_;

    /*! The LWR model
     */
    lwr_lib::LWRPtr lwr_model_;

};

/*! Abbreviation for convinience
 */
typedef LocallyWeightedRegression LWR;
typedef boost::shared_ptr<LWR> LWRPtr;
typedef boost::shared_ptr<LWR const> LWRConstPtr;

bool writeToMessage(const lwr_lib::LWRConstPtr lwr_model,
                    LWRModelMsg& model);

}

#endif /* LOCALLY_WEIGHTED_REGRESSION_H_ */
