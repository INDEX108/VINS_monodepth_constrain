/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../estimator/parameters.h"

class Self_Scale : public ceres::SizedCostFunction<1, 1, 2>
{
  public:
	Self_Scale(const double depth_net);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    double depth_net;
    static double sum_t;
};
