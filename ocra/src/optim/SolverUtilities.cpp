#include "SolverUtilities.h"


namespace ocra
{
  namespace utils
  {
    using namespace details;

    /** \internal Here is the sum up of operations that convert a given [Ax,b] constraint (column entries) 
      * into an [Cx,d] constraint (row entries).
      *
      *          |  Ax+b=0        Ax+b=b'       Ax+b<0        Ax+b<u        0<Ax+b      l<Ax+b          l<Ax+b<u
      * _________|________________________________________________________________________________________________________________
      *          |
      *  Cx+d=0  | C=A, d=b	    C=A, d=b-b'       NA            NA            NA          NA               NA 
      *  Cx-d=0  | C=A, d=-b	  C=A, d=b'-b       NA            NA            NA          NA               NA
      *  Cx+d<0  |    NA            NA        C=A, d=b      C=A, d=b-u    C=-A, d=-b  C=-A, d=l-b   break into l<Ax+b and Ax+b<u
      *  Cx-d<0  |    NA            NA        C=A, d=-b     C=A, d=u-b    C=-A, d=b   C=-A, d=b-l   break into l<Ax+b and Ax+b<u
      *  Cx+d>0  |    NA            NA        C=-A, d=-b    C=-A, d=u-b   C=A, d=b    C=A, d=b-l    break into l<Ax+b and Ax+b<u
      *  Cx-d>0  |    NA            NA        C=-A, d=b     C=-A, d=b-u   C=A, d=-b   C=A, d=l-b    break into l<Ax+b and Ax+b<u
      *  d1<Cx<d2|    NA            NA        C=A, d1=-inf, C=A, d1=-inf, C=A, d1=-b, C=A, d1=l-b,  C=A, d1=l-b, d2=u-b
      *          |                              d2=-b         d2=u-b        d2=+inf     d2=+inf
      *
      * The table below gives the corresponding conversion cases.
      */
    const eConvertCase conversion_cases[7][7] = {
      {PLUS_A_PLUS_B  , PLUS_A_PLUS_BV , IMPOSSIBLE_CASE, IMPOSSIBLE_CASE , IMPOSSIBLE_CASE, IMPOSSIBLE_CASE , IMPOSSIBLE_CASE},
      {PLUS_A_MINUS_B , PLUS_A_MINUS_BV, IMPOSSIBLE_CASE, IMPOSSIBLE_CASE , IMPOSSIBLE_CASE, IMPOSSIBLE_CASE , IMPOSSIBLE_CASE},
      {IMPOSSIBLE_CASE, IMPOSSIBLE_CASE, PLUS_A_PLUS_B  , PLUS_A_PLUS_BV  , MINUS_A_MINUS_B, MINUS_A_MINUS_BV, SPECIAL_CASE   },
      {IMPOSSIBLE_CASE, IMPOSSIBLE_CASE, PLUS_A_MINUS_B , PLUS_A_MINUS_BV , MINUS_A_PLUS_B , MINUS_A_PLUS_BV , SPECIAL_CASE   },
      {IMPOSSIBLE_CASE, IMPOSSIBLE_CASE, MINUS_A_MINUS_B, MINUS_A_MINUS_BV, PLUS_A_PLUS_B  , PLUS_A_PLUS_BV  , SPECIAL_CASE   },
      {IMPOSSIBLE_CASE, IMPOSSIBLE_CASE, MINUS_A_PLUS_B , MINUS_A_PLUS_BV , PLUS_A_MINUS_B , PLUS_A_MINUS_BV , SPECIAL_CASE   },
      {IMPOSSIBLE_CASE, IMPOSSIBLE_CASE, PLUS_A_MINUS_B , PLUS_A_MINUS_BV , PLUS_A_MINUS_B , PLUS_A_MINUS_BV , SPECIAL_CASE   }
    };



    void printSolution(const VectorXd& result, const Variable& var, int space, int precision)
    {
      double eps = 1.e-8;
      double zero = 1.e-15;
      if (var.getNumberOfChildren() >0)
      { 
        const CompositeVariable& cv = reinterpret_cast<const CompositeVariable&>(var);
        int n=0;
        int l;
        for (size_t i=0; i<cv.getNumberOfChildren(); ++i)
        {
          l = cv(i).getSize();
          printSolution(result.segment(n,l), cv(i), space, precision);
          n+=l;
        }
      }
      else
      {
        std::cout << var.getName() << ":  ";
        for (int i=0; i< result.size(); ++i)
        {
          if (fabs(result[i]) < eps)
          {
            if (result[i] > zero)
              std::cout << std::setw(space) << "+eps" << ",";
            else if (result[i] < -zero)
              std::cout << std::setw(space) << "-eps" << ",";
            else
              std::cout << std::setw(space) << "0" << ",";
          }
          else
            std::cout << std::setw(space) << std::setprecision(precision) << result[i] << ",";
        }
      }
      std::cout << std::endl;
    }
  }

  void testUtilities()
  {
    BaseVariable w("t",2);
    BaseVariable x("x",3);
    BaseVariable y("y",3);
    BaseVariable z("z",2);
    CompositeVariable X("X");
    CompositeVariable V("V");
    X.add(w).add(x).add(y).add(z);
    V.add(w).add(y);

    MatrixXd m;
    MatrixXd A = MatrixXd::Zero(12,10);
    MatrixXd Q = MatrixXd::Zero(10,10);
    std::vector<int> mapping;

    m = MatrixXd::Random(4,5);
    utils::uncompressByCol(X, V, m, A.block(3,0,4,10), mapping);
    std::cout << m << std::endl << std::endl;
    std::cout << A << std::endl << std::endl;

    m = MatrixXd::Random(5,5);
    utils::addCompressed2d(m, Q, mapping);
    std::cout << m << std::endl << std::endl;
    std::cout << Q << std::endl << std::endl;

    system("pause");
  }
}


// cmake:sourcegroup=Solvers
