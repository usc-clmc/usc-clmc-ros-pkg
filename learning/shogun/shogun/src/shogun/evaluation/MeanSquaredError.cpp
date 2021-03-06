/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * Written (W) 2011 Sergey Lisitsyn
 * Copyright (C) 2011 Berlin Institute of Technology and Max-Planck-Society
 */

#include <shogun/evaluation/MeanSquaredError.h>
#include <shogun/features/Labels.h>
#include <shogun/mathematics/Math.h>

using namespace shogun;

float64_t CMeanSquaredError::evaluate(CLabels* predicted, CLabels* ground_truth)
{
	ASSERT(predicted->get_num_labels() == ground_truth->get_num_labels());
	int32_t length = predicted->get_num_labels();
	float64_t mse = 0.0;
	for (int32_t i=0; i<length; i++)
		mse += CMath::sq(predicted->get_label(i) - ground_truth->get_label(i));
	mse /= length;
	return mse;
}
