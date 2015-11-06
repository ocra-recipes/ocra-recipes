/**
 * \file OrthonormalFamily.cpp
 * \author Mingxing Liu
 *
 * \brief Orthonormal Family computation for Generalized Hierarchical Control (GHC).
 *
 */

#include "gocra/OrthonormalFamily.h"

using namespace Eigen;

#include <iostream>

namespace gocra
{
OrthonormalFamily::OrthonormalFamily(const MatrixXd& fam, const double eps)
{
    family = fam;
    epsilon = eps;
}

OrthonormalFamily::~OrthonormalFamily()
{

}

void OrthonormalFamily::computeOrthonormalFamily()
{
    int n = family.cols();
    int m = family.rows();
    onf = MatrixXd::Zero(n,n);
    origin = -VectorXi::Zero(n);
    origin.setConstant(-1);

    int i = 0;
    for (int k=0; k<m; ++k)
    {
        if (i>=n)
            break;
        onf.row(i) = family.row(k);
        for (int j=0; j<i;++j)
        {

            onf.row(i)-= onf.row(j) * VectorXd(onf.row(i)).dot(VectorXd(onf.row(j)));
        }
        double nrm = VectorXd(onf.row(i)).norm();
        if (nrm > epsilon)
        {
            onf.row(i) /= nrm;
            origin(i) = k;
            i += 1;
        }

    }
    index = i;

}


int OrthonormalFamily::getIndex() const
{
    return index;
}


const VectorXi& OrthonormalFamily::getOrigin() const
{
    return origin;
}


const MatrixXd& OrthonormalFamily::getOnf() const
{
    return onf;
}


void OrthonormalFamily::setFamily(const MatrixXd& f)
{
    family = f;
}


void OrthonormalFamily::setEpsilon(const double eps)
{
    epsilon = eps;
}
} // namespace gocra
