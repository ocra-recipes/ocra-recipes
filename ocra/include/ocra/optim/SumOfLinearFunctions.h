#ifndef _OCRA_SUM_OF_LINEAR_FUNCTIONS_H_
#define _OCRA_SUM_OF_LINEAR_FUNCTIONS_H_

#include "ocra/optim/LinearFunction.h"
#include "ocra/optim/MergedVariable.h"

#include <vector>

namespace ocra
{
  class SumOfLinearFunctions
    : public LinearFunction
  {
  public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

  private:
    SumOfLinearFunctions(const SumOfLinearFunctions&); // forbid copy
    SumOfLinearFunctions& operator=(const SumOfLinearFunctions&); // forbid assignment

  public:
    SumOfLinearFunctions(int dimension);
    ~SumOfLinearFunctions();

    SumOfLinearFunctions& addFunction(LinearFunction& f, double scale=1.);
    SumOfLinearFunctions& removeFunction(LinearFunction& f, double scale);
    SumOfLinearFunctions& removeFunction(LinearFunction& f);

  protected:
    void updateJacobian() const;
    void updateb() const;
    void doUpdateInputSizeBegin(); // does nothing : this overload allows to resize

    void doChangeA(const MatrixXd& A);
    void doChangeb(const VectorXd& b);

  private:
    void checkThatFunctionsDimensionDoNotChange(int timestamp);

  private:
    typedef std::pair<LinearFunction*, double> weightedFunction_t;
    MergedVariable _variable;
    std::vector<weightedFunction_t> _functions;
    VectorXd _offset;
  };
}

#endif

// cmake:sourcegroup=Function
