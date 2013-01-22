/*********************************************************************
 Computational Learning and Motor Control Lab
 University of Southern California
 Prof. Stefan Schaal
 *********************************************************************
 \remarks      ...

 \file         template_matching.cpp

 \author       Alexander Herzog
 \date         April 1, 2012

 *********************************************************************/
#include <Eigen/StdVector>

#include <grasp_template_planning/template_matching.h>

using namespace std;
using namespace grasp_template;

namespace grasp_template_planning
{

TemplateMatching::TemplateMatching(GraspCreatorInterface const* grasp_creator, boost::shared_ptr<const std::vector<
		  grasp_template::GraspTemplate, Eigen::aligned_allocator<grasp_template::GraspTemplate> > > candidates, boost::shared_ptr<const std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > lib_grasps,
		                     boost::shared_ptr<const std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > > lib_failures,
		                     boost::shared_ptr<const std::vector<std::vector<GraspAnalysis, Eigen::aligned_allocator<GraspAnalysis> > > > lib_successes)
{
  grasp_creator_ = grasp_creator;
  candidates_ = candidates;
  lib_grasps_ = lib_grasps;
  lib_failures_ = lib_failures;
  lib_successes_ = lib_successes;

  lib_scores_.resize((*candidates_).size());
  fail_scores_.resize((*candidates_).size());
  lib_qualities_.resize((*lib_grasps_).size());
  candidate_to_lib_.resize((*candidates_).size());
  candidate_to_succ_.resize((*candidates_).size());
  candidate_to_fail_.resize((*candidates_).size());
  lib_to_fail_.resize((*lib_grasps_).size());

  lib_match_handler_.clear();
  for (unsigned int i = 0; i < (*lib_grasps_).size(); i++)
  {
    const GraspAnalysis& lib_templt = (*lib_grasps_)[i];
    lib_match_handler_.push_back(DismatchMeasure(lib_templt.grasp_template, lib_templt.template_pose.pose,
                                                 lib_templt.gripper_pose.pose));
  }

  lib_succs_match_handler_.clear();
  lib_succ_qualities_.resize((*lib_grasps_).size());
  lib_succ_to_fail_.resize((*lib_grasps_).size());
  ROS_ASSERT((*lib_grasps_).size() == (*lib_successes_).size());
  for (unsigned int i = 0; i < (*lib_successes_).size(); i++)
  {
	  lib_succ_qualities_[i].resize((*lib_successes_)[i].size());
	  lib_succ_to_fail_[i].resize((*lib_successes_)[i].size());
	  lib_succs_match_handler_.push_back(std::vector<grasp_template::DismatchMeasure, Eigen::aligned_allocator<grasp_template::DismatchMeasure> >());
	  for(unsigned int j = 0; j < (*lib_successes_)[i].size(); j++)
	  {
		  const GraspAnalysis& lib_succ_templt = (*lib_successes_)[i][j];
		  lib_succs_match_handler_[i].push_back(DismatchMeasure(lib_succ_templt.grasp_template, lib_succ_templt.template_pose.pose,
				  lib_succ_templt.gripper_pose.pose));
	  }
  }
}

GraspAnalysis TemplateMatching::getGrasp(unsigned int rank) const
{
  const GraspTemplate& templt = (*candidates_)[ranking_[rank]];
  const GraspAnalysis * lib = NULL;
  if(candidate_to_succ_[ranking_[rank]] < 0)
  {
	  lib = &(*lib_grasps_)[candidate_to_lib_[ranking_[rank]]];
  }
  else
  {
	  lib = &(*lib_successes_)[candidate_to_lib_[ranking_[rank]]][candidate_to_succ_[ranking_[rank]]];
  }
  GraspAnalysis res;
  grasp_creator_->createGrasp(templt, *lib, res);
  return res;
}

double TemplateMatching::getScore(unsigned int rank) const
{
  return computeScore(ranking_[rank]);
}

const GraspAnalysis& TemplateMatching::getLib(unsigned int rank) const
{
  return (*lib_grasps_)[candidate_to_lib_[ranking_[rank]]];
}

const GraspAnalysis& TemplateMatching::getPositiveMatch(unsigned int rank) const
{
	if(candidate_to_succ_[ranking_[rank]] < 0)
	{
	  return (*lib_grasps_)[candidate_to_lib_[ranking_[rank]]];
	}
	else
	{
	  return (*lib_successes_)[candidate_to_lib_[ranking_[rank]]][candidate_to_succ_[ranking_[rank]]];
	}
}

string TemplateMatching::getScoreFormula() const
{
  string first = "a/[(1 - exp(- ";
  stringstream ss;
  ss << learningLibQualFac();
  string second;
  ss >> second;
  ss.clear();
  string thirdt = " * b))(1 - exp(- ";
  ss << learningFailDistFac();
  string fourth;
  ss >> fourth;
  ss.clear();
  string fifth = " * c))]";

  string result = first;
  result.append(second);
  result.append(thirdt);
  result.append(fourth);
  result.append(fifth);

  return result;
}

void TemplateMatching::create()
{
  unsigned int lib_index = 0;
#pragma omp parallel for private(lib_index)
  for (lib_index = 0; lib_index < (*lib_grasps_).size(); lib_index++)
  {
    computeLibQuality(lib_index);
  }

  unsigned int cand = 0;
#pragma omp parallel for private(cand)
  for (cand = 0; cand < (*candidates_).size(); cand++)
  {
    TemplateDissimilarity best_cf, best_cl, best_lf;
    double best_m = numeric_limits<double>::max();
    int best_fail_ind = -1;
    unsigned int best_lib_id = 0;
    int best_lib_succ_id = -1;
    for (unsigned int lib = 0; lib < (*lib_grasps_).size(); lib++)
    {
    	const unsigned int num_succs = lib_succs_match_handler_[lib].size();
      TemplateDissimilarity cur_cf, cur_cl, cur_lf;
      vector<TemplateDissimilarity> cur_csucss(num_succs), cur_succf(num_succs);
      double a, b, c, a_occs;
      vector<double> a_succs(num_succs), c_succs(num_succs), a_succs_occs(num_succs);

      //compute m(c, f)
      int cur_fail_index = -1;
      computeFailScore(cand, lib, cur_cf, cur_fail_index);
      if (cur_fail_index >= 0)
      {
        b = cur_cf.getScore();
      }
      else
      {
        b = -1;
      }

      //compute m(c, l)
      {
        GraspTemplate sample((*candidates_)[cand]);
        GraspTemplate sample_tmp = sample;
        const DismatchMeasure& mh = lib_match_handler_[lib];

        mh.applyDcMask(sample_tmp);
        cur_cl = mh.getScore(sample_tmp);

        a = cur_cl.getScore();
        a_occs = cur_cl.getAllFog();

		  // compute m(c, s_i)
		  for(unsigned int suc_ind = 0; suc_ind < num_succs; suc_ind++)
		  {
			  GraspTemplate sample_tmp_i = sample;;
			  const DismatchMeasure& mh = lib_succs_match_handler_[lib][suc_ind];

			  mh.applyDcMask(sample_tmp_i);
			  cur_csucss[suc_ind] = mh.getScore(sample_tmp_i);

			  a_succs[suc_ind] = cur_csucss[suc_ind].getScore();
			  a_succs_occs[suc_ind] = cur_csucss[suc_ind].getAllFog();
		  }
      }

      //compute m(l, f)
      if (lib_to_fail_[lib] >= 0)
      {
        cur_lf = lib_qualities_[lib];
        c = cur_lf.getScore();
      }
      else
      {
        c = -1;
      }
	  //compute m(s_i, f)
      for(unsigned int suc_ind = 0; suc_ind < num_succs; suc_ind++)
      {
    	if (lib_succ_to_fail_[lib][suc_ind] >= 0)
		{
    	  cur_succf[suc_ind] = lib_succ_qualities_[lib][suc_ind];
		  c_succs[suc_ind] = cur_succf[suc_ind].getScore();
		}
		else
		{
		  c_succs[suc_ind] = -1;
	    }
      }

      double m = computeScore(a, b, c, a_occs);
      vector<double> m_succs(num_succs);
      m_succs.resize(lib_succs_match_handler_[lib].size());
      for(unsigned int suc_ind = 0; suc_ind < num_succs; suc_ind++)
      {
    	  m_succs[suc_ind] = computeScore(a_succs[suc_ind], b, c_succs[suc_ind], a_succs_occs[suc_ind]);
      }

      if (m < best_m)
      {
        best_m = m;
        best_cl = cur_cl;
        best_cf = cur_cf;
//        best_lf = cur_lf;
        best_fail_ind = cur_fail_index;
        best_lib_id = lib;
        best_lib_succ_id = -1;
      }
      for(unsigned int suc_ind = 0; suc_ind < num_succs; suc_ind++)
      {
    	  if (m_succs[suc_ind] < best_m)
    	  {
			best_m = m_succs[suc_ind];
			best_cl = cur_csucss[suc_ind];
			best_cf = cur_cf;
//	        best_lf = cur_lf;
			best_fail_ind = cur_fail_index;
			best_lib_id = lib;
			best_lib_succ_id = suc_ind;
    	  }
      }
    }

    candidate_to_lib_[cand] = best_lib_id;
    candidate_to_succ_[cand] = best_lib_succ_id;
    candidate_to_fail_[cand] = best_fail_ind;
    lib_scores_[cand] = best_cl;
    fail_scores_[cand] = best_cf;
  }

  map<double, unsigned int> ranking_map;
  for (unsigned int i = 0; i < (*candidates_).size(); i++)
  {
    double score = computeScore(i);
    //map does not store replicants!!!!!! -> candidates.size() != ranking_.size()
    ranking_map.insert(make_pair<double, unsigned int> (score, i));
  }

  ranking_.clear();
  for (map<double, unsigned int>::const_iterator it = ranking_map.begin(); it != ranking_map.end(); it++)
  {
    ranking_.push_back(it->second);
  }
}

//bool TemplateMatching::exceedsDissimilarityThreshold(const GraspAnalysis& succ_demo) const
//{
//  GraspTemplate sample(succ_demo.grasp_template, succ_demo.template_pose.pose);
//  TemplateDissimilarity score;
//  unsigned int lib_index = -1;
//  computeLibScore(sample, score, lib_index);
//
//  return score.getScore() > learningSuccAddDist();
//}

void TemplateMatching::writeScores(ostream& stream, unsigned int max_scores) const
{
  stream << "filename" << "\t" << "lib_score" << "\t" << "first_s" << "\t" << "second_s" << "\t" << "ss" << endl;
  for (unsigned int i = 0; i < max_scores && i < size(); i++)
  {
    stream << getLib(i).demo_filename;
    if(candidate_to_succ_[ranking_[i]] >= 0)
    {
    	stream << " further succ";
    }
    stream << "\t";
    stream << getScore(i) << "\t";
    stream << getLibOverlay(i) << "\t";
    stream << lib_scores_[ranking_[i]].sd_ + lib_scores_[ranking_[i]].sf_ + lib_scores_[ranking_[i]].ss_
        + lib_scores_[ranking_[i]].st_ << "\t";
    stream << lib_scores_[ranking_[i]].ds_ + lib_scores_[ranking_[i]].fs_ + lib_scores_[ranking_[i]].ss_
        + lib_scores_[ranking_[i]].ts_ << "\t";
    stream << lib_scores_[ranking_[i]].ss_;
    stream << endl;
  }
}

//void TemplateMatching::computeLibScore(GraspTemplate& candidate, TemplateDissimilarity& score, unsigned int index) const
//{
//  for (unsigned int i = 0; i < (*lib_grasps_).size(); i++)
//  {
//    const DismatchMeasure& mh = lib_match_handler_[i];
//
//    mh.applyDcMask(candidate);
//    TemplateDissimilarity cur = mh.getScore(candidate);
//
//    if (i == 0 || cur.isBetter(cur, score))
//    {
//      score = cur;
//      index = i;
//    }
//  }
//}

void TemplateMatching::computeLibQuality(unsigned int lib_index)
{
  // Demonstration against Failures
  TemplateDissimilarity closest;
  int fail_index = -1;

  if (lib_failures_ != NULL)
  {
    for (unsigned int i = 0; i < (*lib_failures_)[lib_index].size(); i++)
    {
      GraspTemplate f_templt((*lib_failures_)[lib_index][i].grasp_template,
                             (*lib_failures_)[lib_index][i].template_pose.pose);
      lib_match_handler_[lib_index].applyDcMask(f_templt);

      TemplateDissimilarity cur = lib_match_handler_[lib_index].getScore(f_templt);
      if (i == 0 || cur.isBetter(cur, closest))
      {
        closest = cur;
        fail_index = i;
      }
    }
  }

  lib_qualities_[lib_index] = closest;
  lib_to_fail_[lib_index] = fail_index;

  // Lib succs against Failures
  std::vector<TemplateDissimilarity>& succ_quals =  lib_succ_qualities_[lib_index];
  std::vector<grasp_template::DismatchMeasure, Eigen::aligned_allocator<grasp_template::DismatchMeasure> >& succ_match_handler = lib_succs_match_handler_[lib_index];
  if (lib_failures_ != NULL)
  {
	  for(unsigned int i = 0; i < succ_quals.size(); i++)
	  {
		  closest = TemplateDissimilarity();
		  fail_index = -1;

		  for (unsigned int j = 0; j < (*lib_failures_)[lib_index].size(); j++)
		  {
			  GraspTemplate f_templt((*lib_failures_)[lib_index][j].grasp_template,
									 (*lib_failures_)[lib_index][j].template_pose.pose);

			  succ_match_handler[i].applyDcMask(f_templt);

			  TemplateDissimilarity cur = succ_match_handler[i].getScore(f_templt);
			  if (j == 0 || cur.isBetter(cur, closest))
			  {
				 closest = cur;
				 fail_index = j;
			  }
		  }

		  succ_quals[i] = closest;
		  lib_succ_to_fail_[lib_index][i] = fail_index;
	  }
  }
}

void TemplateMatching::computeFailScore(unsigned int candidate, unsigned int lib_index, TemplateDissimilarity& score,
                                        int& fail_index) const
{
  GraspTemplate sample((*candidates_)[candidate]);
  lib_match_handler_[lib_index].applyDcMask(sample);

  fail_index = -1;

  if (lib_failures_ != NULL)
  {
    for (unsigned int i = 0; i < (*lib_failures_)[lib_index].size(); i++)
    {
      GraspTemplate f_templt((*lib_failures_)[lib_index][i].grasp_template,
                             (*lib_failures_)[lib_index][i].template_pose.pose);
      lib_match_handler_[lib_index].applyDcMask(f_templt);

      TemplateDissimilarity cur = lib_match_handler_[lib_index].getScore(sample, f_templt);
      if (i == 0 || cur.isBetter(cur, score))
      {
        score = cur;
        fail_index = i;
      }
    }
  }
}

double TemplateMatching::computeScore(double a, double b, double c, double occlusions) const
{
  if (b >= 0.0)
  {
    b = 1.0 - exp(-learningFailDistFac() * b * b);
  }
  else
    b = 1.0;

  if (c >= 0.0)
  {
    c = 1.0 - exp(-learningLibQualFac() * c * c);
  }
  else
    c = 1.0;

  occlusions = 0.0;//occlusions / TemplateHeightmap::TH_DEFAULT_NUM_TILES_X / TemplateHeightmap::TH_DEFAULT_NUM_TILES_X;
  if(occlusions >= 0.0)
  {
	  occlusions = 2.0 - exp(-occlusionPunishmentFac() * occlusions * occlusions);
  }
  else
	  occlusions = 1.0;

  if (b < 0.0001)
    b = 0.0001;
  if (c < 0.0001)
    c = 0.0001;

  return (a * occlusions) / b / c;
}

double TemplateMatching::getLibQuality(unsigned int rank) const
{
	if(candidate_to_succ_[ranking_[rank]] < 0)
	{
		return lib_qualities_[candidate_to_lib_[ranking_[rank]]].getScore();
	}
	else
	{
		return lib_succ_qualities_[candidate_to_lib_[ranking_[rank]]][candidate_to_succ_[ranking_[rank]]].getScore();
	}
};

double TemplateMatching::computeScore(unsigned int cand) const
{
  const double a = lib_scores_[cand].getScore(); //m(c, l) is to minimize!
  double occlusions = 0.0;//lib_scores_[cand].getAllFog() / TemplateHeightmap::TH_DEFAULT_NUM_TILES_X / TemplateHeightmap::TH_DEFAULT_NUM_TILES_X;

  if(occlusions >= 0.0)
    {
  	  occlusions = 2.0 - exp(-occlusionPunishmentFac() * occlusions * occlusions);
    }
    else
  	  occlusions = 1.0;

  double b; //m(c, f); is to be maximized!
  if (candidate_to_fail_[cand] >= 0)
  {
    b = fail_scores_[cand].getScore();
    b = 1 - exp(-learningFailDistFac() * b * b);
  }
  else
    b = 1;

  double c; //m(l, f); is to be maximized!
  if (lib_to_fail_[candidate_to_lib_[cand]] >= 0)
  {
	if(candidate_to_succ_[cand] < 0)
	{
		c = lib_qualities_[candidate_to_lib_[cand]].getScore();
		c = 1 - exp(-learningLibQualFac() * c * c);
	}
	else
	{
		c = lib_succ_qualities_[candidate_to_lib_[cand]][candidate_to_succ_[cand]].getScore();
		c = 1 - exp(-learningLibQualFac() * c * c);
	}
  }
  else
    c = 1;

  return(a * occlusions) / b / c;
}

double TemplateMatching::getLibOverlay(unsigned int rank) const
{

  return lib_scores_[ranking_[rank]].getMinOverlay();
}

} //namespace
