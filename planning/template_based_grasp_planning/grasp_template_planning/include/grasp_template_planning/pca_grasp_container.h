/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         pca_grasp_container.h

 \author       Alexander Herzog
 \date         July 29, 2012

 *********************************************************************/

#ifndef PCA_GRASP_CONTAINER_H_
#define PCA_GRASP_CONTAINER_H_

#include <grasp_template_planning/GraspAnalysis.h>
#include <grasp_template_planning/template_matching.h>

namespace grasp_template_planning
{

class PCAGraspContainer : public TemplateMatching
{
public:
	PCAGraspContainer(GraspCreatorInterface const* grasp_creator);
	virtual ~PCAGraspContainer(){};
	  virtual unsigned int size() const {return grasps_.size();};
	  virtual double getLibScore(unsigned int rank) const {return 0.0;};
	  virtual double getFailScore(unsigned int rank) const {return 0.0;};
	  virtual double getLibQuality(unsigned int rank) const {return 0.0;};
	  virtual double getHypothesisScore(unsigned int rank) const {return 0.0;};
	  virtual double getScore(unsigned int rank) const {return 0.0;};
	  virtual void writeScores(std::ostream& stream, unsigned int max_scores) const{};
	  virtual void create(){};

	  virtual GraspAnalysis getGrasp(unsigned int rank) const;
	  virtual const GraspAnalysis& getLib(unsigned int rank) const;
	  virtual const GraspAnalysis& getPositiveMatch(unsigned int rank) const;

	//  bool exceedsDissimilarityThreshold(const GraspAnalysis& succ_demo) const;

	  std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > grasps_;
	  GraspAnalysis fake_lib_;
private:
};
}  //namespace
#endif /* PCA_GRASP_CONTAINER_H_ */
