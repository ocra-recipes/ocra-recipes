#include "ocra/optim/utilities.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/QuadraticSolver.h"



namespace ocra
{
  int ToScilab::precision = Eigen::StreamPrecision;

  ToScilab::ToScilab(const Variable& v)
  {
    Eigen::IOFormat f(precision, 0, ", ", ";", "", "", "[", "]");
    std::ostringstream os;
    os << v.getValue().format(f);
    str = os.str();
  }

  ToScilab::ToScilab(const QuadraticSolver& s)
  {
    throw "TODO";
  }

  std::ostream& operator<< (std::ostream& os, const ToScilab& obj)
  {
    return os << obj.str;
  }
}

// cmake:sourcegroup=Utils
