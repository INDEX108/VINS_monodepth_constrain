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

#include "Self_Scale.h"
#include <cmath>

double Self_Scale::sum_t;

Self_Scale::Self_Scale(const double depth_net) :depth_net(depth_net)
{
};

bool Self_Scale::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;

    double inv_dep_i = parameters[0][0];
    double dep_i = 1.0 / inv_dep_i;
    double ak = parameters[1][0];
    double real_ak = ak;//1e-5+std::log(1+std::exp(ak));
    double bk = parameters[1][1];

    Eigen::Map<Eigen::Matrix<double, 1, 1>> residual(residuals);

    double real_depth;
    real_depth = real_ak *  depth_net + bk;
    residual(0, 0) = std::log(real_depth*dep_i);

    //printf("residual: %f\n", residual(0, 0));

    if (jacobians)
    {

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 1>> inv_depth_jac(jacobians[0]);
            inv_depth_jac << -1/(dep_i);
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 1, 2>> jacobian_shift(jacobians[1]);
            jacobian_shift << (depth_net)/(real_depth*dep_i) , 1/(real_depth*dep_i);
        }

    }
    sum_t += tic_toc.toc();

    return true;
}
