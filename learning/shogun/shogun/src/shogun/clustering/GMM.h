/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Alesis Novik
 * Copyright (C) 2011 Berlin Institute of Technology and Max-Planck-Society
 */
#ifndef _GMM_H__
#define _GMM_H__

#include <shogun/lib/config.h>

#ifdef HAVE_LAPACK

#include <shogun/distributions/Distribution.h>
#include <shogun/distributions/Gaussian.h>
#include <shogun/lib/common.h>

namespace shogun
{
/** @brief Gaussian Mixture Model interface.
 *
 * Takes input of number of Gaussians to fit and a covariance type to use.
 */
class CGMM : public CDistribution
{
	public:
		/** default constructor */
		CGMM();
		/** constructor
		 *
		 * @param n number of Gaussians
		 * @param cov_type covariance type
		 */
		CGMM(int32_t n, ECovType cov_type=FULL);
		/** constructor
		 *
		 * @param components GMM components
		 * @param coefficients mixing coefficients
		 * @param copy true if should be copied
		 */
		CGMM(SGVector<CGaussian*> components, SGVector<float64_t> coefficients, bool copy=false);
		virtual ~CGMM();

		/** cleanup */
		void cleanup();

		/** set training data for use with EM or SMEM
		 *
		 * @param data training data
		 *
		 * @return true
		 */
		virtual bool train(CFeatures* data=NULL);

		/** learn model using EM
		 *
		 * @param min_cov minimum covariance
		 * @param max_iter maximum iterations
		 * @param min_change minimum change in log likelihood
		 *
		 * @return log likelihood of training data
		 */
		float64_t train_em(float64_t min_cov=1e-9, int32_t max_iter=1000, float64_t min_change=1e-9);

		/** learn model using SMEM
		 *
		 * @param max_iter maximum SMEM iterations
		 * @param max_cand maximum split-merge candidates
		 * @param min_cov minimum covariance
		 * @param max_em_iter maximum iterations for EM
		 * @param min_change minimum change in log likelihood
		 *
		 * @return log likelihood of training data
		 */
		float64_t train_smem(int32_t max_iter=100, int32_t max_cand=5, float64_t min_cov=1e-9, int32_t max_em_iter=1000, float64_t min_change=1e-9);

		/** maximum likelihood estimation
		 *
		 * @param alpha point assignment
		 * @param min_cov minimum covariance
		 */
		void max_likelihood(SGMatrix<float64_t> alpha, float64_t min_cov);

		/** get number of parameters in model
		 *
		 * @return number of parameters in model
		 */
		virtual int32_t get_num_model_parameters();

		/** get model parameter (logarithmic)
		 *
		 * @return model parameter (logarithmic) if num_param < m_dim returns
		 * an element from the mean, else return an element from the covariance
		 */
		virtual float64_t get_log_model_parameter(int32_t num_param);

		/** get partial derivative of likelihood function (logarithmic)
		 *
		 * @param num_param derivative against which param
		 * @param num_example which example
		 * @return derivative of likelihood (logarithmic)
		 */
		virtual float64_t get_log_derivative(
			int32_t num_param, int32_t num_example);

		/** compute log likelihood for example
		 *
		 * abstract base method
		 *
		 * @param num_example which example
		 * @return log likelihood for example
		 */
		virtual float64_t get_log_likelihood_example(int32_t num_example);

		/** compute likelihood for example
		 *
		 * abstract base method
		 *
		 * @param num_example which example
		 * @return likelihood for example
		 */
		virtual float64_t get_likelihood_example(int32_t num_example)
		{
			return CMath::exp(get_log_likelihood_example(num_example));
		}

		/** get nth mean
		 *
		 * @param num index of mean to retrieve
		 *
		 * @return mean
		 */
		virtual inline SGVector<float64_t> get_nth_mean(int32_t num)
		{
			ASSERT(num<m_components.vlen);
			return m_components.vector[num]->get_mean();
		}

		/** set nth mean
		 *
		 * @param mean new mean
		 * @param num index mean to set
		 */
		virtual inline void set_nth_mean(SGVector<float64_t> mean, int32_t num)
		{
			ASSERT(num<m_components.vlen);
			m_components.vector[num]->set_mean(mean);
		}

		/** get nth covariance
		 *
		 * @param num index of covariance to retrieve
		 *
		 * @return covariance
		 */
		virtual inline SGMatrix<float64_t> get_nth_cov(int32_t num)
		{
			ASSERT(num<m_components.vlen);
			return m_components.vector[num]->get_cov();
		}

		/** set nth covariance
		 *
		 * @param cov new covariance
		 * @param num index of covariance to set
		 */
		virtual inline void set_nth_cov(SGMatrix<float64_t> cov, int32_t num)
		{
			ASSERT(num<m_components.vlen);
			m_components.vector[num]->set_cov(cov);
		}

		/** get coefficients
		 *
		 * @return coeffiecients
		 */
		virtual inline SGVector<float64_t> get_coef()
		{
			return m_coefficients;
		}

		/** set coefficients
		 *
		 * @param coefficients mixing coefficients
		 */
		virtual inline void set_coef(SGVector<float64_t> coefficients)
		{
			m_coefficients.destroy_vector();
			m_coefficients=coefficients;
		}

		/** get components
		 *
		 * @return components
		 */
		virtual inline SGVector<CGaussian*> get_comp()
		{
			return m_components;
		}

		/** set components
		 *
		 * @param components Gaussian components
		 */
		virtual inline void set_comp(SGVector<CGaussian*> components)
		{
			for (int32_t i=0; i<m_components.vlen; i++)
			{
				SG_UNREF(m_components.vector[i]);
			}

			m_components.destroy_vector();
			m_components=components;

			for (int32_t i=0; i<m_components.vlen; i++)
			{
				SG_REF(m_components.vector[i]);
			}
		}

		/** sample from model
		 *
		 * @return sample
		 */
		SGVector<float64_t> sample();

		/** cluster point
		 *
		 * @return log likelihood of belonging to clusters and the log likelihood of being generated by this GMM
		 * The length of the returned vector is number of components + 1
		 */
		SGVector<float64_t> cluster(SGVector<float64_t> point);

		/** @return object name */
		inline virtual const char* get_name() const { return "GMM"; }

	private:
		/** 1NN assignment initialization 
		 *
		 * @param init_means initial means
		 *
		 * @return initial alphas
		 */
		SGMatrix<float64_t> alpha_init(SGMatrix<float64_t> init_means);

		/** Initialize parameters for serialization */
		void register_params();

		/** apply the partial EM algorithm on 3 components 
		 *
		 * @param comp1 index of first component
		 * @param comp2 index of second component
		 * @param comp3 index of third component
		 * @param min_cov minimum covariance
		 * @param max_em_iter maximum iterations for EM
		 * @param min_change minimum change in log likelihood
		 */
		void partial_em(int32_t comp1, int32_t comp2, int32_t comp3, float64_t min_cov, int32_t max_em_iter, float64_t min_change);

	protected:
		/** Mixture components */
		SGVector<CGaussian*> m_components;
		/** Mixture coefficients */
		SGVector<float64_t> m_coefficients;
};
}
#endif //HAVE_LAPACK
#endif //_GMM_H__
