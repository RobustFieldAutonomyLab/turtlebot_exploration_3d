#ifndef COVMATERNISO3_H
#define COVMATERNISO3_H

#include "Eigen/Dense"

/**
 * @brief Matern3 kernel.
          cov = sf2*(1+sqrt(3)/ell*d)*exp(-sqrt(3)/ell*d),
          in which d is the Euclidean distance of two points.
 *
 * @param x m by d(imension) input vector.
 * @param z n by d(imension) input vector.
 * @param sf2 prior variance.
 * @param ell lengthscale.
 * @param diag if diag is true, only the diagonal of covariance matrix is
               returned.
 *
 * @return covariance matrix (or its diagonal).
 */
Eigen::MatrixXf covMaterniso3(const Eigen::MatrixXf &x, const Eigen::MatrixXf &z, double sf2, double ell, bool diag=false);

/**
 * @brief Euclidean distance between two vectors.
          dist = [d(x1, z1) d(x1, z2) ... d(x1, zn)
                              ......
                  d(xm, z1) d(xm, z2) ... d(xm, zn)]
 *
 * @param x 1 by m vector.
 * @param z 1 by n vector.
 *
 * @return m by n distance matrix.
 */
Eigen::MatrixXf dist(const Eigen::MatrixXf &x, const Eigen::MatrixXf &z);

#endif // COVMATERNISO3_H
