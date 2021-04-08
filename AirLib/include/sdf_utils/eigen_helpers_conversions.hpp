#include <Eigen/Geometry>
#include <Eigen/Jacobi>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
#include <sdf_utils/pretty_print.hpp>
#include <functional>
#include "eigen_typedefs.hpp"

#ifndef EIGEN_HELPERS_CONVERSIONS_HPP
#define EIGEN_HELPERS_CONVERSIONS_HPP

namespace EigenHelpersConversions
{
    inline Eigen::Matrix3Xd VectorEigenVector3dToEigenMatrix3Xd(const EigenHelpers::VectorVector3d& vector_eigen)
    {
        Eigen::Matrix3Xd eigen_matrix(3, vector_eigen.size());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            eigen_matrix.col(idx) = vector_eigen[idx];
        }
        return eigen_matrix;
    }

    inline EigenHelpers::VectorVector3d EigenMatrix3XdToVectorEigenVector3d(const Eigen::Matrix3Xd& eigen_matrix)
    {
        EigenHelpers::VectorVector3d vector_eigen(eigen_matrix.cols());
        for (size_t idx = 0; idx < vector_eigen.size(); idx++)
        {
            vector_eigen[idx] = eigen_matrix.col(idx);
        }
        return vector_eigen;
    }
}

#endif // EIGEN_HELPERS_CONVERSIONS_HPP
