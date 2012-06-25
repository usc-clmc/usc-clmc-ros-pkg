/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks		...
 
 \file		parameters.h

 \author	Peter Pastor
 \date		Nov 4, 2010

 *********************************************************************/

#ifndef LWR_PARAMETERS_BASE_H_
#define LWR_PARAMETERS_BASE_H_

// system includes
#include <string>
#include <vector>
#include <Eigen/Core>

#include <boost/shared_ptr.hpp>

// local includes
#include <lwr_lib/status.h>

namespace lwr_lib
{

class LWRParameters : public Status
{

    /*! Allow the LocallyWeightedRegression class to access private member variables directly
     */
    friend class LWR;

public:

    /*! Constructor
     */
    LWRParameters() :
      num_rfs_(0) {};

    /*! Destructor
     */
    virtual ~LWRParameters() {};

    /*!
     * @param lwr_model
     * @return True if equal, otherwise False
     */
    bool operator==(const LWRParameters& params) const
    {
      return ((isInitialized() && params.isInitialized())
          && (num_rfs_ == params.num_rfs_)
          && (fabs((centers_ - params.centers_).norm()) < EQUALITY_PRECISSION)
          && (fabs((widths_ - params.widths_).norm()) < EQUALITY_PRECISSION)
          && (fabs((slopes_ - params.slopes_).norm()) < EQUALITY_PRECISSION)
          && (fabs((offsets_ - params.offsets_).norm()) < EQUALITY_PRECISSION) );
    }
    bool operator!=(const LWRParameters& params) const
    {
      return !(*this == params);
    }

    /*!
     * @param file_name
     * @return True on success, otherwise False
     */
    // bool initialize(const std::string& file_name);

    /*!
     * @param num_rfs
     * @param exponentially_spaced Determines whether gaussians are exponentially spaced (true) or equally spaced (false).
     *  Exponential scale is used to deal with the nonlinearity of the
     *  phase variable of the canonical system of a DMP
     * @param cutoff
     * @return True on success, otherwise False
     */
    bool initialize(const int num_rfs, const double activation,
                    bool exponentially_spaced = true, const double cutoff = 0.001);

    /*!
     * @param centers
     * @param widths
     * @param slopes
     * @param offsets
     * @return True on success, otherwise False
     */
    bool initialize(const Eigen::VectorXd& centers,
                    const Eigen::VectorXd& widths,
                    const Eigen::VectorXd& slopes,
                    const Eigen::VectorXd& offsets);

    /*!
     * @param centers
     * @param widths
     * @param slopes
     * @param offsets
     * @return True on success, otherwise False
     */
    bool initialize(const std::vector<double>& centers,
                    const std::vector<double>& widths,
                    const std::vector<double>& slopes,
                    const std::vector<double>& offsets);

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
     * @return
     */
    bool setNumRFS(const int num_rfs);

    /*! Returns the number of receptive fields
     * @return
     */
    int getNumRFS() const;

    /*! Writes the LWR model to file
     *
     * @param file_name File name in which the LWR model is stored.
     * @return True on success, false on failure
     */
    bool writeToDisc(const std::string& file_name);

    /*! Return a string containing information about the LWR model
     * @return string containing information about the LWR model
     */
    std::string getInfoString() const;

private:

    const static double EQUALITY_PRECISSION = 1e-6;

    /*! Number of receptive fields used in this LWR model
     */
    int num_rfs_;

    /*! Centers of the receptive fields
     */
    Eigen::VectorXd centers_;

    /*! Bandwidth used for each local model
     */
    Eigen::VectorXd widths_;

    /*! Slopes of the local linear approximations
     */
    Eigen::VectorXd slopes_;

    /*! Offsets of the local linear approximations. (currently not implemented)
     */
    Eigen::VectorXd offsets_;

};


/*! Abbreviation for convinience
 */
typedef LWRParameters LWRParam;
typedef boost::shared_ptr<LWRParam> LWRParamPtr;
typedef boost::shared_ptr<LWRParam const> LWRParamConstPtr;

}

#endif /* LWR_PARAMETERS_BASE_H_ */
