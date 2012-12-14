/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         planning_pipeline.h

 \author       Alexander Herzog
 \date         July 29, 2012

 *********************************************************************/

#ifndef PCA_PLANNING_PIPE_H_
#define PCA_PLANNING_PIPE_H_

#include <pcl/common/pca.h>
#include <grasp_template/grasp_template.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template_planning/planning_pipeline.h>

namespace grasp_template_planning
{

class GraspAnaComparableWrapper
{
public:
	GraspAnalysis ana_;
	geometry_msgs::Pose table_pose_;

	GraspAnaComparableWrapper(const GraspAnalysis& ana, const geometry_msgs::Pose& table_pose) : ana_(ana),
	table_pose_(table_pose){};
	bool operator <(const GraspAnaComparableWrapper& b) const;
};

class PCAPlanningPipe : public PlanningPipeline
{
public:
	PCAPlanningPipe(const std::string& demo_path,
		    const std::string& library_path, const std::string& failures_path,
		    const std::string& successes_path, const std::string& log_data_path);
	virtual ~PCAPlanningPipe();

	 virtual bool addFailure(const GraspAnalysis& lib_grasp, const GraspAnalysis& failure){return false;};
	  virtual bool addSuccess(const GraspAnalysis& lib_grasp, const GraspAnalysis& success){return false;};

//	  virtual void createGrasp(const grasp_template::GraspTemplate& templt,
//	        const GraspAnalysis& lib_grasp, GraspAnalysis& result) const;
	    virtual void planGrasps(boost::shared_ptr<TemplateMatching>& pool) const;
//	    virtual bool logPlannedGrasps(const TemplateMatching& pool, unsigned int max_num_grasps);
//	    virtual bool logGraspResult(const GraspAnalysis& res_ana, const TemplateMatching& pool, int rank);
//	    virtual bool writeLogToBag();

private:
	    void poseEigenToTf(const Eigen::Matrix4f& transform,
	    		geometry_msgs::PoseStamped& pose) const;
	    void shiftOutOfConvexHull(const Eigen::Vector3f& mean,
	    		const Eigen::Vector3f& shift_dir, Eigen::Vector3f& shift_back) const;
};
}  //namespace
#endif /* PCA_PLANNING_PIPE_H_ */
