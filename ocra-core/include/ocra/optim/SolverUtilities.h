#ifndef _OCRA_SOLVER_UTILITIES_H_
#define _OCRA_SOLVER_UTILITIES_H_

#ifdef WIN32
# pragma once
#endif

#include "ocra/MathTypes.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/uncompress.h"

#include <Eigen/Eigen>
namespace ocra{using namespace Eigen;}

#include <iostream>
#include <iomanip>

namespace ocra
{
  enum eReturnInfo
  {
    RETURN_SUCCESS = 0,            //< lucky you
    RETURN_INCONSISTENT_PROBLEM,   //< the problem does not meet the solver requirements
    RETURN_INFEASIBLE_PROBLEM,     //< the problem was decided to be infeasible by the solver
    RETURN_MAX_ITER_REACHED,       //< the maximum number of iterations was reached before a solution was found
    RETURN_MAX_TIME_REACHED,       //< the maximum time allowed was reached before a solution was found
    RETURN_MEMORY_ERROR,           //< the solver was not able to allocate all the memory needed, or the provided memory was not enough
    RETURN_NUMERICAL_ERROR,        //< some numerical errors occured which prevented the solution to be found
    RETURN_ERROR,                  //< other/undetermined
  };


  struct OptimizationResult
  {
    eReturnInfo info;       //< error code
    VectorXd    solution;   //< the solution if inform is 0. If not, it depends on the solver
  };


  /** Enumerates the possible forms of a linear constraint */
  enum eConstraintOutput
  {
    CSTR_PLUS_EQUAL=0,  // Cx+d=0
    CSTR_MINUS_EQUAL,   // Cx-d=0
    CSTR_PLUS_LOWER,    // Cx+d<0
    CSTR_MINUS_LOWER,   // Cx-d<0
    CSTR_PLUS_GREATER,  // Cx+d>0
    CSTR_MINUS_GREATER, // Cx-d>0
    CSTR_DOUBLE_BOUNDS  // d1<Cx<d2
  };


  namespace utils
  {
    namespace details
    {
      /** \internal never change this enum
        * Enumerates the operations that must be done to convert a constraint Ax+b. v is a vector.
        */
      enum eConvertCase
      {
        PLUS_A_PLUS_B=0,  //C=A,   d=b
        PLUS_A_MINUS_B,   //C=A,   d=-b
        PLUS_A_PLUS_BV,   //C=A,   d=b-v
        PLUS_A_MINUS_BV,  //C=A,   d=v-b
        MINUS_A_PLUS_B,   //C=-A,  d=b
        MINUS_A_MINUS_B,  //C=-A,  d=-b
        MINUS_A_PLUS_BV,  //C=-A,  d=b-v
        MINUS_A_MINUS_BV, //C=-A,  d=v-b
        SPECIAL_CASE,
        IMPOSSIBLE_CASE,
      };
    }

    /** \internal a sum up of the possible conversion cases. conversion_cases[i][j] returns the eConvertCase value 
      * corresponding to the ith type of constraint conversion forms (ith element of eConstraintOutput) and the jth
      * element of the constraint types (jth element of eConstraintType).
      */
    extern const details::eConvertCase conversion_cases[7][7];

    /** Print in a formated way the coefficient of a linear equation described by \a A and \a b*/
    template<class Derived, class VectorBase>
    inline void printLinearEquation(const MatrixBase<Derived>& A, const VectorBase& b, int space = 9, int precision=3);

    /** Print the solution stored in \a result according to the form of the variable \a var*/
    void printSolution(const VectorXd& result, const Variable& var, int space = 9, int precision=3);

    /** The following functions are aliases on usages of the methods of uncompress<Functor> (Uncompress.h), defined for
      * userfriendliness:
      * - uncompressByCol
      * - uncompressByRow
      * - uncompress2d
      * - addCompressedByCol
      * - addCompressedByRow
      * - addCompressed2d
      * - minCompressedByCol
      * - minCompressedByRow
      * - minCompressed2d
      * - maxCompressedByCol
      * - maxCompressedByRow
      * - maxCompressed2d
      * each of them having two overloads
      * void functionName(const MatrixBase<Derived1>& in, MatrixBase<Derived2>& out,
      *                   const std::vector<int>& mapping, double scale=1., bool reverseMapping);
      *
      * void functionName(const Variable& base, const Variable& rel,
      *               const MatrixBase<Derived1>& in, MatrixBase<Derived2>& out,
      *               std::vector<int>& mapping, double scale=1.)
      *
      * Given two matrices \a in and \a out, and a vector of indices \a mapping, these functions will perform an
      * operation between:
      *  - in.col(i) and out.col(mapping[i]) for functions postfixed by \a ByRow,
      *  - in.row(j) and out.row(mapping[j]) for functions postfixed by \a ByCol,
      *  - in(i,j) and out(mapping[i], mapping[j]) for functions postfixed by \a 2d,
      * for i in [0,in.cols()[ or/and j in [0,in.rows()[.
      * If \a reverseMapping is set to \c true, the indices k and mapping[k] are swaped: for example, for functions
      * postfixed by \a ByRow, the operation will happen between in.col(mapping[i]) and out.col(i)
      * The operation between these two elements \a e1 and \a e2 is
      * e2 = e1 for \a uncompress functions,
      * e2 += e1 for \a addCompressed functions,
      * e2 = min(e1,e2) for \a minCompressed functions,
      * e2 = max(e1,e2) for \a maxCompressed functions.
      *
      * Each function has two overloads: 
      *  - const (MatrixBase&, MatrixBase&, const vector<int>&, double, bool reverseMapping) for which the mapping is 
      * given by the user.
      *  - const (const Variable&, constVariable& MatrixBase&, MatrixBase&, vector<int>&, double) for which the 
      * mapping is computed from the variables (base.getRelativeMappingOf(rel, mapping)) and returned to the user.
      *
      * \internal Having a reversed mapping is possible for the second function prototype too, but was not done yet. It
      * requires to write the corresponding assertions and to compute the mapping of base wrt to rel, in which case the
      * names of the variables does not seem very suitable.
      */
    //@{
      
    // const_cast Eigen trick : http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
#define DECLARE_UNCOMPRESSED_FUNCTION(functionName) \
    template<class Derived1, class Derived2> \
    void functionName(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out, \
                      const std::vector<int>& mapping, double scale=1., bool reverseMapping=false); \
    template<class Derived1, class Derived2>  \
    void functionName(const Variable& base, const Variable& rel, \
                      const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out, \
                      std::vector<int>& mapping, double scale=1.); \
    template<typename Scalar, class Derived1, class Derived2> \
    /* specialization for when in is of the form a*M */ \
    inline void functionName(const CwiseUnaryOp<internal::scalar_multiple_op<Scalar>, Derived1>& in, MatrixBase<Derived2> const& _out, \
                      const std::vector<int>& mapping, double scale=1., bool reverseMapping=false) \
    { functionName(in.nestedExpression(), _out, mapping, scale*in.functor().m_other, reverseMapping); } \
    template<typename Scalar, class Derived1, class Derived2>  \
    void functionName(const Variable& base, const Variable& rel, \
                      const CwiseUnaryOp<internal::scalar_multiple_op<Scalar>, Derived1>& in, MatrixBase<Derived2> const& _out, \
                      std::vector<int>& mapping, double scale=1.) \
    { \
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2> & >(_out);\
      functionName(base, rel, in.nestedExpression(), out, mapping, scale*in.functor().m_other); \
    }

    DECLARE_UNCOMPRESSED_FUNCTION(uncompressByCol)
    DECLARE_UNCOMPRESSED_FUNCTION(uncompressByRow)
    DECLARE_UNCOMPRESSED_FUNCTION(uncompress2d)
    DECLARE_UNCOMPRESSED_FUNCTION(addCompressedByCol)
    DECLARE_UNCOMPRESSED_FUNCTION(addCompressedByRow)
    DECLARE_UNCOMPRESSED_FUNCTION(addCompressed2d)
    DECLARE_UNCOMPRESSED_FUNCTION(minCompressedByCol)
    DECLARE_UNCOMPRESSED_FUNCTION(minCompressedByRow)
    DECLARE_UNCOMPRESSED_FUNCTION(minCompressed2d)
    DECLARE_UNCOMPRESSED_FUNCTION(maxCompressedByCol)
    DECLARE_UNCOMPRESSED_FUNCTION(maxCompressedByRow)
    DECLARE_UNCOMPRESSED_FUNCTION(maxCompressed2d)
    //@}


    /*Uncompress and convert a linear constraint*/
    template<class Derived, class VectorBase1, class VectorBase2>
    void convert(const LinearConstraint& cstr, const std::vector<int>& mapping, eConstraintOutput type,
                 MatrixBase<Derived>& A, VectorBase1& b, VectorBase2& l, double infinity = 0.);
    
    /** Intersects the set described by \a bounds with the one already described by \a bl and \a bu and put the result
      * into \a bl and \a bu.
      * \a bounds must be either a IdentityConstraint or a BoundConstraint
      */
    template<class VectorBase1, class VectorBase2>
    void intersectBounds(const DiagonalLinearConstraint& bounds, const std::vector<int>& mapping, VectorBase1& bl, VectorBase2& bu);
  }

  void testUtilities();
}


#include "SolverUtilities.hxx"

#endif // _OCRA_SOLVER_UTILITIES_H_

// cmake:sourcegroup=Solvers
