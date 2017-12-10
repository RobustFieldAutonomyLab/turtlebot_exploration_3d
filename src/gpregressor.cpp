#ifdef MKL
#include "mkl.h"
#endif

#include "gpregressor.h"
#include "covMaterniso3.h"
// #include <iostream>


using namespace Eigen;
// using namespace std;

void GPRegressor::train(const MatrixXf &_x, const MatrixXf &y) {
    x = MatrixXf(_x);
    K = covMaterniso3(x, x, sf2, ell);
    K = K + noise * MatrixXf::Identity(K.rows(), K.cols());
    LLT<MatrixXf> llt(K);
    alpha = llt.solve(y);
    L = llt.matrixL();
}

void GPRegressor::test(const MatrixXf &xs, MatrixXf &m, MatrixXf &s2) const {
    MatrixXf Ks = covMaterniso3(x, xs, sf2, ell);
    m = Ks.transpose() * alpha;
#ifdef MKL
    int n, nrhs;
    n = L.rows();
    nrhs = Ks.cols();
    double *ap = (double *)mkl_malloc(n * (n + 1) / 2 * sizeof(double), 64);
    double *b = Ks.data();
    for (int i = 0, k = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) {
            ap[k++] = L(j, i);
        }
    }
    LAPACKE_dtptrs(LAPACK_COL_MAJOR, 'L', 'N', 'N', n, nrhs, ap, b, n);
    mkl_free(ap);
    Map<MatrixXf> v(b, n, nrhs);
#else
    MatrixXf v = L.triangularView<Lower>().solve(Ks);
#endif // MKL

    MatrixXf Kss = covMaterniso3(xs, xs, sf2, ell, true);
    s2 = Kss - (v.transpose() * v).diagonal();
}

//void GPRegressor::bcm(const MatrixXf &m1, const MatrixXf &s21, MatrixXf &m2,
//                      MatrixXf &s22) const {
//    MatrixXf is21 = s21.cwiseInverse();
//    MatrixXf is22 = s22.cwiseInverse();
//    m2 = (is21.array() * m1.array() + is22.array() * m2.array()).matrix();
//    s22 = (is21.array() + is22.array() - sf2).cwiseInverse();
//    m2 = (s22.array() * m2.array()).matrix();
//}


// int main()
// {
//     GPRegressor g(1, 1, 1);
//     MatrixXf x(2, 2), y(2, 1), xs(2, 2);
//     x << 1, 2, 2, 3;
//     y << 1, -1;
//     xs(0,0) = 1;  xs(0,1) = 2;
//     xs(1,0) = 2;  xs(1,1) = 3;

//     MatrixXf m, s2;
//     g.train(x, y);
//     g.test(xs, m, s2);
//     std::cout << "gp mean  : " << m << "size: " << m.size() << std::endl;
//     std::cout << "gp cov   : " << s2 << std::endl;

//     return 0;
// }
