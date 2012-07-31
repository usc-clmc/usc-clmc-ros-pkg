/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         pca_grasp_container.cpp

 \author       Alexander Herzog
 \date         July 29, 2012

 *********************************************************************/

#include <grasp_template/grasp_template.h>
#include <grasp_template/grasp_template_params.h>
#include <grasp_template_planning/planning_pipeline.h>
#include <grasp_template_planning/pca_grasp_container.h>

using namespace grasp_template;

namespace grasp_template_planning
{
PCAGraspContainer::PCAGraspContainer(GraspCreatorInterface const* grasp_creator) :

		 TemplateMatching(grasp_creator, boost::shared_ptr<const std::vector<
				  grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > >(new std::vector<
						  grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> >()),
				  boost::shared_ptr<const std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > >(new std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> >()),
				  boost::shared_ptr<const std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > >(new std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > >()),
				  boost::shared_ptr<const std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > >(new std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > >()))
{
}

GraspAnalysis PCAGraspContainer::getGrasp(unsigned int rank) const
{
	return grasps_[rank];
}
const GraspAnalysis& PCAGraspContainer::getLib(unsigned int rank) const
{
	return fake_lib_;
}
const GraspAnalysis& PCAGraspContainer::getPositiveMatch(unsigned int rank) const
{
	return fake_lib_;
}
}
