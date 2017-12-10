#include "covMaterniso3.h"

using namespace Eigen;

MatrixXf dist(const MatrixXf &x, const MatrixXf &z) {
    MatrixXf d(x.rows(), z.rows());
    for(int i=0; i<x.rows(); ++i) {
        d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
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

