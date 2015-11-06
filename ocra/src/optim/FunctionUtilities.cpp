#include "ocra/optim/FunctionUtilities.h"
#include "ocra/optim/Variable.h"
#include <sstream>


namespace ocra
{
  namespace utils
  {
    int getAddType(const Function& f1, const Function& f2)
    {
      int i = f1.getType(); 
      int j = f2.getType();

      if (i==j)
        return i;

      if (i>j)
      {
        int e = i;
        i = j;
        j = e;
      }

      switch (j)
      {
        case LINEARITY_UNDEFINED:
          return LINEARITY_UNDEFINED;
        case LINEARITY_LINEAR_FRACTIONAL:
          if (i==LINEARITY_CONSTANT && i==LINEARITY_LINEAR)
            return LINEARITY_LINEAR_FRACTIONAL;
          break;
        case LINEARITY_QUADRATIC:
          if (i==LINEARITY_CONSTANT && i==LINEARITY_LINEAR)
            return LINEARITY_QUADRATIC;
          break;
        case LINEARITY_LINEAR:
          if (i==LINEARITY_CONSTANT)
            return LINEARITY_LINEAR;
          break;
        default:
          return LINEARITY_UNDEFINED;
        }
        return LINEARITY_UNDEFINED;
    }


    int getAddContinuityProperty(const Function& f1, const Function& f2)
    {
      return std::min(f1.getContinuityProperty(), f2.getContinuityProperty());
    }

    bool computeGradient(const Function& f1, const Function& f2)
    {
      return f1.canCompute<PARTIAL_X>() && f2.canCompute<PARTIAL_X>();
    }

    bool computeHessian(const Function& f1, const Function& f2)
    {
      return f1.canCompute<PARTIAL_XX>() && f2.canCompute<PARTIAL_XX>();
    }

    /*Variable* getConcatenatedVariable(Function& f1, Function& f2)
    {
    const std::string& n1 = f1.getVariable().getName();
    const std::string& n2 = f2.getVariable().getName();
    std::string name = n1+n2;

    CompositeVariable* v = new CompositeVariable(name, f1.getVariable());

    v->addByMerge(f2.getVariable());
    return v;
    }*/

    int getOppositeConvexityProperty(const Function& f)
    {
      switch (f.getConvexityProperty())
      {
        case CONVEXITY_CONVEX :
          return CONVEXITY_CONCAVE;

        case CONVEXITY_STRICTLY_CONVEX :    
          return CONVEXITY_STRICTLY_CONCAVE;

        case CONVEXITY_CONCAVE :
          return CONVEXITY_CONVEX;

        case CONVEXITY_STRICTLY_CONCAVE :
          return CONVEXITY_STRICTLY_CONVEX;

        case CONVEXITY_CONVEX_AND_CONCAVE :
          return CONVEXITY_CONVEX_AND_CONCAVE;

        case CONVEXITY_LOG_CONVEX :
          return CONVEXITY_UNDEFINED;

        case CONVEXITY_LOG_CONCAVE :
          return CONVEXITY_UNDEFINED;

        case CONVEXITY_QUASICONVEX :
          return CONVEXITY_QUASICONCAVE;

        case CONVEXITY_QUASICONCAVE :
          return CONVEXITY_QUASICONVEX;

        case CONVEXITY_QUASILINEAR :
          return CONVEXITY_QUASILINEAR;

        default:
          return CONVEXITY_UNDEFINED;
      }
    }


    Variable* createOutputVariable(Function& f)
    {
      static int cpt = 0;
      std::stringstream s;
      s << "undefVar";
      s << cpt++;
      return new BaseVariable(s.str(), f.getDimension());
    }
  }
}

// cmake:sourcegroup=Function
