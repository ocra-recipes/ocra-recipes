#ifndef _OCRA_SOLVER_UTILITIES_HXX_
#define _OCRA_SOLVER_UTILITIES_HXX_

#ifdef WIN32
# pragma once
#endif

namespace ocra
{
  namespace utils
  {
    template<class Derived, class VectorBase>
    inline void printLinearEquation(const MatrixBase<Derived>& A, const VectorBase& b, int space, int precision)
    {
      OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase);
      ocra_assert(b.rows()==1 || b.cols()==1);
      double eps = 1.e-8;
      double zero = 1.e-15;
      ocra_assert(A.rows() == b.size());

      for (int r=0; r<A.rows(); ++r)
      {
        if (r%10 == 0)
        {
          for (int c=0; c<A.cols(); ++c)
          {
            if (c%10 == 0)
              std::cout << (c/10)%10 << "-";
            else
              std::cout << "-";
            for (int i=0; i<space; ++i)
              std::cout << "-";
          }
          std::cout << std::endl;
        }

        for (int c=0; c<A.cols(); ++c)
        {
          if (c%10 == 0)
            std::cout << "|";
          if (std::abs(A(r,c)) < eps)
          {
            if (A(r,c)>zero)
              std::cout << std::setw(space) << "+eps";
            else if (A(r,c)<-zero)
              std::cout << std::setw(space) << "-eps";
            else
              std::cout << std::setw(space) << "0";
          }
          else
          {
            if (std::abs(A(r,c)) < 0.01)
              std::cout << std::setw(space) << std::setprecision(precision-1) << A(r,c);
            else
              std::cout <<  std::setw(space) << std::setprecision(precision) << A(r,c);
          }
          if (c<A.cols()-1)
            std::cout << ",";
        }

        std::cout << "      ";
        if (std::abs(b[r]) < eps)
        {
          if (b[r]>zero)
            std::cout << std::setw(space) << "+eps" << std::endl;
          else if (b[r]<-zero)
            std::cout << std::setw(space) << "-eps" << std::endl;
          else
            std::cout << std::setw(space) << "0" << std::endl;
        }
        else
          std::cout << std::setw(space) << std::setprecision(precision) << b[r] << std::endl;
      }

      std::cout << std::endl;
    }

/*  uncompressedByCol
    uncompressedByRow
    uncompressed2d*/

    template<class Derived1, class Derived2>
    inline void uncompressByCol(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                      const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<assign_functor, false>::uncompressByCol(in, out, mapping, scale);
      else
        uncompress<assign_functor, true>::uncompressByCol(in, out, mapping, scale);

    }

    // const_cast Eigen trick : http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
    template<class Derived1, class Derived2>
    inline void uncompressByCol(const Variable& base, const Variable& rel,
                      const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                      std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.rows() == _out.rows());
      ocra_assert(_out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::uncompressByCol] base variable does not contains relative variable" );

      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      uncompressByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void uncompressByRow(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                      const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<assign_functor, false>::uncompressByRow(in, out, mapping, scale);
      else
        uncompress<assign_functor, true>::uncompressByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void uncompressByRow(const Variable& base, const Variable& rel,
                      const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                      std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.cols() == out.cols());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::uncompressByRow] base variable does not contains relative variable" );

      uncompressbyRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void uncompress2d(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                      const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<assign_functor, false>::uncompress2d(in, out, mapping, mapping, scale);
      else
        uncompress<assign_functor, true>::uncompress2d(in, out, mapping, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void uncompress2d(const Variable& base, const Variable& rel,
                      const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                      std::vector<int>& mapping, double scale)
    {
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::uncompress2d] base variable does not contains relative variable" );

      uncompress2d(in, out, mapping, scale);
    }



/*  addCompressedByCol
    addCompressedByRow
    addCompressed2d*/

    template<class Derived1, class Derived2>
    inline void addCompressedByCol(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<add_functor, false>::uncompressByCol(in, out, mapping, scale);
      else
        uncompress<add_functor, true>::uncompressByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void addCompressedByCol(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.rows() == out.rows());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::addCompressByCol] base variable does not contains relative variable" );

      addCompressByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void addCompressedByRow(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<add_functor, false>::uncompressByRow(in, out, mapping, scale);
      else
        uncompress<add_functor, true>::uncompressByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void addCompressedByRow(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.cols() == out.cols());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::addCompressedByRow] base variable does not contains relative variable" );

      addCompressedByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void addCompressed2d(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                       const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<add_functor, false>::uncompress2d(in, out, mapping, mapping, scale);
      else
        uncompress<add_functor, true>::uncompress2d(in, out, mapping, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void addCompressed2d(const Variable& base, const Variable& rel,
                       const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                       std::vector<int>& mapping, double scale)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::addCompressed2d] base variable does not contains relative variable" );

      addCompressed2d(in, out, mapping, scale);
    }



/*  minCompressedByCol
    minCompressedByRow
    minCompressed2d*/

    template<class Derived1, class Derived2>
    inline void minCompressedByCol(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<min_functor, false>::uncompressByCol(in, out, mapping, scale);
      else
        uncompress<min_functor, true>::uncompressByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void minCompressedByCol(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.rows() == out.rows());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::minCompressedByCol] base variable does not contains relative variable" );

      minCompressedByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void minCompressedByRow(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<min_functor, false>::uncompressByRow(in, out, mapping, scale);
      else
        uncompress<min_functor, true>::uncompressByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void minCompressedByRow(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.cols() == out.cols());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::minCompressedByRow] base variable does not contains relative variable" );

      minCompressedByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void minCompressed2d(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                       const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<min_functor, false>::uncompress2d(in, out, mapping, mapping, scale);
      else
        uncompress<min_functor, true>::uncompress2d(in, out, mapping, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void minCompressed2d(const Variable& base, const Variable& rel,
                       const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                       std::vector<int>& mapping, double scale)
    {
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::minCompressed2d] base variable does not contains relative variable" );

      minCompressed2d(in, out, mapping, scale);
    }


/*  maxCompressedByCol
    maxCompressedByRow
    maxCompressed2d*/

    template<class Derived1, class Derived2>
    inline void maxCompressedByCol(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<max_functor, false>::uncompressByCol(in, out, mapping, scale);
      else
        uncompress<max_functor, true>::uncompressByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void maxCompressedByCol(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.rows() == out.rows());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::maxCompressedByCol] base variable does not contains relative variable" );

      maxCompressedByCol(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void maxCompressedByRow(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                          const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<max_functor, false>::uncompressByRow(in, out, mapping, scale);
      else
        uncompress<max_functor, true>::uncompressByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void maxCompressedByRow(const Variable& base, const Variable& rel,
                          const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                          std::vector<int>& mapping, double scale)
    {
      ocra_assert(in.cols() == out.cols());
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::maxCompressedByRow] base variable does not contains relative variable" );

      maxCompressedByRow(in, out, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void maxCompressed2d(const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& _out,
                       const std::vector<int>& mapping, double scale, bool reverseMapping)
    {
      MatrixBase<Derived2>& out = const_cast<MatrixBase<Derived2>& >(_out);
      if (reverseMapping)
        uncompress<max_functor, false>::uncompress2d(in, out, mapping, mapping, scale);
      else
        uncompress<max_functor, true>::uncompress2d(in, out, mapping, mapping, scale);
    }

    template<class Derived1, class Derived2>
    inline void maxCompressed2d(const Variable& base, const Variable& rel,
                       const MatrixBase<Derived1>& in, MatrixBase<Derived2> const& out,
                       std::vector<int>& mapping, double scale)
    {
      ocra_assert(out.cols() == base.getSize());
      ocra_assert(in.cols() == rel.getSize());

      base.getRelativeMappingOf(rel, mapping);
      if (mapping.size() != rel.getSize())
        throw std::runtime_error("[ocra::utils::maxCompressed2d] base variable does not contains relative variable" );

      maxCompressed2d(in, out, mapping, scale);
    }


    namespace details
    {
      // const_cast Eigen trick : http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html#TopicPlainFunctionsFailing
      template<class Derived, class VectorDerived>
      void convert_(const LinearConstraint& cstr, eConstraintType apparentType, const std::vector<int>& mapping, eConvertCase convertCase,
                    MatrixBase<Derived>& A, MatrixBase<VectorDerived> const & _b)
      {
        VectorXd dummy;
        const VectorXd* v;
        switch (apparentType)
        {
          case CSTR_EQUAL_B:    v = &cstr.getB(); break;
          case CSTR_LOWER_U:    v = &cstr.getU(); break;
          case CSTR_GREATER_L:  v = &cstr.getL(); break;
          default: 
            v = &dummy;
        }

        MatrixBase<VectorDerived>& b = const_cast<MatrixBase<VectorDerived>& >(_b);
        convert(cstr.getFunction().getA(), cstr.getFunction().getb(), *v, A, b, mapping, convertCase);
      }


      template<class DerivedIn, class VectorBaseIn1, class VectorBaseIn2, class DerivedOut, class VectorBaseOut>
      inline void convert(const MatrixBase<DerivedIn>& M, const VectorBaseIn1& v1, const VectorBaseIn2& v2,
                          MatrixBase<DerivedOut>& A, VectorBaseOut& b,
                          const std::vector<int>& mapping,
                          details::eConvertCase convertCase)
      {
        OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBaseIn1);
        ocra_assert(v1.cols()==1);
        OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBaseIn2);
        ocra_assert(v2.cols()==1);
        ocra_assert(M.rows()== A.rows() && "Input and output matrices do not have the same row size");
        ocra_assert(v1.size()== b.size() && "Input and output vectors do not have the same size");

        switch (convertCase)
        {
          case details::PLUS_A_PLUS_B:    uncompressByCol(M, A, mapping,  1.);   b = v1;     break;
          case details::PLUS_A_MINUS_B:   uncompressByCol(M, A, mapping,  1.);   b = -v1;    break;
          case details::PLUS_A_PLUS_BV:   uncompressByCol(M, A, mapping,  1.);   b = v1-v2;  break;
          case details::PLUS_A_MINUS_BV:  uncompressByCol(M, A, mapping,  1.);   b = v2-v1;  break;
          case details::MINUS_A_PLUS_B:   uncompressByCol(M, A, mapping, -1.);   b = v1;     break;     
          case details::MINUS_A_MINUS_B:  uncompressByCol(M, A, mapping, -1.);   b = -v1;    break;     
          case details::MINUS_A_PLUS_BV:  uncompressByCol(M, A, mapping, -1.);   b = v1-v2;  break;     
          case details::MINUS_A_MINUS_BV: uncompressByCol(M, A, mapping, -1.);   b = v2-v1;  break;
          default:
            ocra_assert(false && "this should never happen");
        }
      }
    } // details


    template<class Derived, class VectorBase1, class VectorBase2>
    inline void convert(const LinearConstraint& cstr, const std::vector<int>& mapping, eConstraintOutput type,
                 MatrixBase<Derived>& A, VectorBase1& b, VectorBase2& l, double infinity)
    {
      details::eConvertCase cvrtCase = conversion_cases[type][cstr.getType()];

      ocra_assert(cvrtCase != details::IMPOSSIBLE_CASE && "You cannot do this conversion.");

      VectorXd dummy;
      if (cvrtCase != details::SPECIAL_CASE)
      {
        if (type != CSTR_DOUBLE_BOUNDS)
        {
          details::convert_(cstr, cstr.getType(), mapping, cvrtCase, A, b);
        }
        else
        {
          ocra_assert(infinity > 0 && "if you use this type of output (CSTR_DOUBLE_BOUNDS), please specify its value (last argument of convert())");
          switch (cstr.getType())
          {
            case CSTR_LOWER_ZERO: 
              details::convert(cstr.getFunction().getA(), cstr.getFunction().getb(), dummy, A, b, mapping, cvrtCase); l.fill(-infinity); 
              break;
            case CSTR_LOWER_U:    
              details::convert(cstr.getFunction().getA(), cstr.getFunction().getb(), cstr.getU(), A, b, mapping, cvrtCase); l.fill(-infinity); 
              break;
            case CSTR_GREATER_ZERO:
              details::convert(cstr.getFunction().getA(), cstr.getFunction().getb(), dummy, A, l, mapping, cvrtCase); b.fill(infinity);
              break;
            case CSTR_GREATER_L: 
              details::convert(cstr.getFunction().getA(), cstr.getFunction().getb(), cstr.getL(), A, l, mapping, cvrtCase); b.fill(infinity);
              break;
            default:
              ocra_assert(false && "this should never happen");
          }
        }
      }
      else
      {
        if (type != CSTR_DOUBLE_BOUNDS)
        {
          int dim = cstr.getDimension();
          ocra_assert(A.rows() == 2*dim && "Output matrix A does not have the correct number of rows");
          ocra_assert(b.size() == 2*dim && "Output vector b does not have the correct dimension");
          Eigen::Block<Derived> A_block_0 = A.block(0, 0, dim, A.cols());
          Eigen::Block<Derived> A_block_dim = A.block(dim, 0, dim, A.cols());

          switch (type)
          {
            case CSTR_PLUS_LOWER:
              details::convert_(cstr, CSTR_LOWER_U, mapping, details::PLUS_A_PLUS_BV,  A_block_0, b.head(dim)); 
              details::convert_(cstr, CSTR_GREATER_L, mapping, details::MINUS_A_MINUS_BV, A_block_dim, b.tail(dim)); 
              break;
            case CSTR_MINUS_LOWER:
              details::convert_(cstr, CSTR_LOWER_U, mapping, details::PLUS_A_MINUS_BV, A_block_0, b.head(dim));
              details::convert_(cstr, CSTR_GREATER_L, mapping, details::MINUS_A_PLUS_BV, A_block_dim, b.tail(dim));
              break;
            case CSTR_PLUS_GREATER:
              details::convert_(cstr, CSTR_LOWER_U, mapping, details::MINUS_A_MINUS_BV, A_block_0, b.head(dim)); 
              details::convert_(cstr, CSTR_GREATER_L, mapping, details::PLUS_A_PLUS_BV,   A_block_dim, b.tail(dim));
              break;
            case CSTR_MINUS_GREATER:
              details::convert_(cstr, CSTR_LOWER_U, mapping, details::MINUS_A_PLUS_BV, A_block_0, b.head(dim)); 
              details::convert_(cstr, CSTR_GREATER_L, mapping, details::PLUS_A_MINUS_BV, A_block_dim, b.tail(dim));
              break;
            default:
              ocra_assert(false && "this should never happen");
          }
        }
        else
        {
          details::convert(cstr.getFunction().getA(), cstr.getFunction().getb(), cstr.getU(), A, b, mapping, details::PLUS_A_MINUS_BV); 
          l = cstr.getL() - cstr.getFunction().getb();        
        }
      }
    }


    template<class VectorBase1, class VectorBase2>
    inline void intersectBounds(const DiagonalLinearConstraint& bounds, const std::vector<int>& mapping, VectorBase1& bl, VectorBase2& bu)
    {
      OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase1);
      ocra_assert(bl.rows()==1 || bl.cols()==1);
      OCRA_STATIC_ASSERT_VECTOR_OR_DYNAMIC_MATRIX(VectorBase2);
      ocra_assert(bu.rows()==1 || bu.cols()==1);
      ocra_assert((dynamic_cast<const BoundConstraint*>(&bounds) || dynamic_cast<const IdentityConstraint*>(&bounds)) && "bounds can only be of type BoundConstraint or IdentityConstraint");

      //TODO [done]: corriger pour les DiagonalLinearFunction avec -I (bornes inf)
      //-> TODO: validate

      if (bounds.getDimension() == 0)
        return;
      if (bounds.getFunction().getA()(0,0) > 0) //i.e. A is the identity
      {
        switch(bounds.getType())
        {
          case CSTR_LOWER_ZERO:         utils::minCompressedByRow(              - bounds.getFunction().getb(), bu, mapping); break;
          case CSTR_LOWER_U:            utils::minCompressedByRow(bounds.getU() - bounds.getFunction().getb(), bu, mapping); break;
          case CSTR_GREATER_ZERO:       utils::maxCompressedByRow(              - bounds.getFunction().getb(), bl, mapping); break;
          case CSTR_GREATER_L:          utils::maxCompressedByRow(bounds.getL() - bounds.getFunction().getb(), bl, mapping); break;
          case CSTR_LOWER_AND_GREATER:  utils::minCompressedByRow(bounds.getU() - bounds.getFunction().getb(), bu, mapping);
                                        utils::maxCompressedByRow(bounds.getL() - bounds.getFunction().getb(), bl, mapping); break;
          default:
            ocra_assert(false && "should never happen, the constraint should be a bound constraint, hence an inequality");
        }
      }
      else //i.e. A is the -I
      {
        switch(bounds.getType())
        {
          case CSTR_LOWER_ZERO:         utils::maxCompressedByRow(bounds.getFunction().getb()                , bl, mapping); break;
          case CSTR_LOWER_U:            utils::maxCompressedByRow(bounds.getFunction().getb() - bounds.getU(), bl, mapping); break;
          case CSTR_GREATER_ZERO:       utils::minCompressedByRow(bounds.getFunction().getb()                , bu, mapping); break;
          case CSTR_GREATER_L:          utils::minCompressedByRow(bounds.getFunction().getb() - bounds.getL(), bu, mapping); break;
          case CSTR_LOWER_AND_GREATER:  utils::maxCompressedByRow(bounds.getFunction().getb() - bounds.getU(), bl, mapping);
                                        utils::minCompressedByRow(bounds.getFunction().getb() - bounds.getL(), bu, mapping); break;
          default:
            ocra_assert(false && "should never happen, the constraint should be a bound constraint, hence an inequality");
        }
      }
    }
  }
}

#endif // _OCRA_SOLVER_UTILITIES_HXX_

// cmake:sourcegroup=Solvers
