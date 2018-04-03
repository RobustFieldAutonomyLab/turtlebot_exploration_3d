#include "covMaterniso3.h"

using namespace Eigen;
const double twoPI = 3.14159265*2;

MatrixXf dist(const MatrixXf &x, const MatrixXf &z) {
    // update on Jan 18th 2018 by Bona, rewrite the dist between two vector taking the wrap around on yaw angle

    MatrixXf d(x.rows(), z.rows()), temp(z.rows(), z.cols());
    for(int i=0; i<x.rows(); ++i) {
        // commented old dist, when no wrap around considered.
        //d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();

        // new dist, with wrap around on yaw angle, last element here, also assuming all the angles within [0,2PI]
        temp = z.rowwise() - x.row(i);
        for(int j=0;j<temp.rows();j++) {
            float d_c  = std::abs(z(j,z.cols()-1) - x(i,x.cols()-1)); // clockwise
            float d_cc = std::min(z(j,z.cols()-1), x(i,x.cols()-1)) + std::min(twoPI-z(j,z.cols()-1), twoPI-x(i,x.cols()-1)); //counter clockwise
            temp(j, temp.cols()-1) = std::min(d_c, d_cc);
        }

        d.row(i) = temp.rowwise().norm();
    }
    return d;
}

MatrixXf covMaterniso3(const MatrixXf &x, const MatrixXf &z, double sf2, double ell, bool diag) {
    MatrixXf K;
    if(diag) {
        K = MatrixXf::Zero(x.rows(), 1);
    } else {
        K = dist(1.73205/ell/ell*x, 1.73205/ell/ell*z);
    }
    return ((1+K.array())*exp(-K.array())).matrix()*sf2;
}

