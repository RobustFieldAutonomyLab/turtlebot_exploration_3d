#ifndef GPOCTOMAP_GPREGRESSOR_H
#define GPOCTOMAP_GPREGRESSOR_H

#include "Eigen/Dense"

using namespace Eigen;

class GPRegressor {
public:
    GPRegressor(double sf2_, double ell_, double noise_) :
            sf2(sf2_), ell(ell_), noise(noise_) {
    }
    void train(const Eigen::MatrixXf &x, const Eigen::MatrixXf &y);
    void test(const Eigen::MatrixXf &xs, Eigen::MatrixXf &m, Eigen::MatrixXf &s2) const;



private:
    double sf2;
    double ell;
    double noise;
    Eigen::MatrixXf x;
    Eigen::MatrixXf K;
    Eigen::MatrixXf alpha;
    Eigen::MatrixXf L;
};
#endif //GPOCTOMAP_GPREGRESSOR_H

