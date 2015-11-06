#ifndef __ORTHONORMALFAMILY_H__
#define __ORTHONORMALFAMILY_H__

#include <Eigen/Eigen>
namespace gocra
{

class OrthonormalFamily
{
public:
    OrthonormalFamily(const Eigen::MatrixXd& family, const double epsilon);
    ~OrthonormalFamily();
    void computeOrthonormalFamily();
    int getIndex() const;
    const Eigen::VectorXi& getOrigin() const;
    const Eigen::MatrixXd& getOnf() const;
    void setFamily(const Eigen::MatrixXd& family);
    void setEpsilon(const double epsilon);

private:
    int index;
    Eigen::VectorXi origin;
    Eigen::MatrixXd onf;
    Eigen::MatrixXd family;
    double epsilon;

};


/** \} */ // end group core


}

#endif
