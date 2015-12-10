/** @file Uncompress.h
 *  @brief Set structures for uncompress untility.
 *
 *         Copyright (C) 2010 CEA/DRT/LIST/DIASI/LSI
 *
 *  @author Escande Adrien
 *  @date 10.06.28
 */



#ifndef _OCRA_UTILS_UNCOMPRESS_H_
#define _OCRA_UTILS_UNCOMPRESS_H_

#ifdef WIN32
# pragma once
#endif

#include <Eigen/Eigen>
namespace ocra{using namespace Eigen;}

namespace ocra
{
  namespace utils
  {
    template <class T, class U>
    struct specialized_min
    {
      // to understand a bit more the following return type
      //typedef const Eigen::CwiseBinaryOp<Eigen::internal::scalar_min_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<V>, const Eigen::ArrayWrapper<W> > return_t;
      //typedef Eigen::CwiseUnaryOp<Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<V>::Scalar>, Eigen::ArrayWrapper<W> > mult_t;
      //typedef const Eigen::CwiseBinaryOp<Eigen::internal::scalar_min_op<typename Eigen::internal::traits<V>::Scalar>, Eigen::ArrayWrapper<V>, mult_t > return2_t;

      template <class V, class W>
      static const Eigen::CwiseBinaryOp<Eigen::internal::scalar_min_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<V>, const Eigen::ArrayWrapper<W> >
        run(const V& a, const W& b) { return a.array().min(b.array()); }

      template <class V, class W>
      static const Eigen::CwiseBinaryOp<Eigen::internal::scalar_min_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<V>, const Eigen::CwiseUnaryOp<Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<T>::Scalar>, const Eigen::ArrayWrapper<W> > >
        run(const V& a, double alpha, const W& b) { return a.array().min(alpha*b.array()); }
    };

    template <>
    struct specialized_min<double, double>
    {
      static const double& run(const double& a, const double& b) { return std::min(a, b); }
      static const double& run(const double& a, double alpha, const double& b) { return std::min(a, alpha*b); }
    };


    template <class T, class U>
    struct specialized_max
    {
      //typedef const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<typename Eigen::internal::traits<T>::Scalar>, Eigen::ArrayWrapper<T>, Eigen::ArrayWrapper<U> > return_t;
      //typedef Eigen::CwiseUnaryOp<Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<T>::Scalar>, Eigen::ArrayWrapper<U> > mult_t;
      //typedef const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<typename Eigen::internal::traits<T>::Scalar>, Eigen::ArrayWrapper<T>, mult_t > return2_t;

      template <class V, class W>
      static const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<V>, const Eigen::ArrayWrapper<W> > 
        run(const V& a, const W& b) { return a.array().max(b.array()); }
      template <class V, class W>
      static const Eigen::CwiseBinaryOp<Eigen::internal::scalar_max_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<V>, const Eigen::CwiseUnaryOp<Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<V>::Scalar>, const Eigen::ArrayWrapper<W> > > 
        run(const V& a, double alpha, const W& b) { return a.array().max(alpha*b.array()); }
    };

    template <>
    struct specialized_max<double, double>
    {
      static const double& run(const double& a, const double& b) { return std::max(a, b); }
      static const double& run(const double& a, double alpha, const double& b) { return std::max(a, alpha*b); }
    };


    template<class T, class U>
    struct assign_functor
    {
      static void run(T& out, const U& in) { out = in; }
      static void run(T& out, double alpha, const U& in) { out = alpha*in; }
    };

    template<class T, class U>
    struct add_functor
    {
      static void run(T& out, const U& in) { out += in; }
      static void run(T& out, double alpha, const U& in) { out += alpha*in; }
    };

    template<class T, class U>
    struct min_functor
    {
      static void run(T& out, const U& in) { out = specialized_min<T,U>::run(out, in); }
      static void run(T& out, double alpha, const U& in) { out = specialized_min<T,U>::run(out, alpha, in); }
    };

    template<class T, class U>
    struct max_functor
    {
      static void run(T& out, const U& in) { out = specialized_max<T,U>::run(out, in); }
      static void run(T& out, double alpha, const U& in) { out = specialized_max<T,U>::run(out, alpha, in); }
    };



    template<template <class T, class U> class Functor, bool ApplyMappingToOut = true>
    struct uncompress
    {
      template<typename ArgType>
      static ArgType selector(ArgType i, ArgType j) { return ApplyMappingToOut?i:j; }

      template<class Derived1, class Derived2>
      static void uncompressByCol(const MatrixBase<Derived1>& in, MatrixBase<Derived2>& out, 
                                  const std::vector<int>& mapping, double scale)
      {
        typedef typename MatrixBase<Derived1>::ConstColXpr col1_t;
        typedef typename MatrixBase<Derived2>::ColXpr col2_t;

        const int N = static_cast<int>( selector(in.cols(), out.cols()) );
        ocra_assert(in.rows() == out.rows());
        ocra_assert(N == mapping.size());
        if (scale == 1.)
        {
          for (int i=0; i<N; ++i) {
            col2_t c2 = out.col(selector(mapping[i],i));
            Functor<col2_t, col1_t>::run(c2,
                                         in.col(selector(i, mapping[i])));
          }
        }
        else
        {
          for (int i=0; i<N; ++i) {
            col2_t c2 = out.col(selector(mapping[i],i));
            Functor<col2_t, col1_t>::run(c2, 
                                         scale,
                                         in.col(selector(i, mapping[i])));
          }
        }
      }

      template<class Derived1, class Derived2>
      static void uncompressByRow(const MatrixBase<Derived1>& in, MatrixBase<Derived2>& out, 
                                  const std::vector<int>& mapping, double scale)
      {
        typedef typename MatrixBase<Derived1>::ConstRowXpr row1_t;
        typedef typename MatrixBase<Derived2>::RowXpr row2_t;

        const int N = static_cast<int>( selector(in.rows(), out.rows()) );
        ocra_assert(in.cols() == out.cols());
        ocra_assert(N == mapping.size());
        if (scale == 1.)
        {
          for (int i=0; i<N; ++i) {
            row2_t r2 = out.row(selector(mapping[i],i));
            Functor<row2_t, row1_t>::run(r2, 
                                         in.row(selector(i, mapping[i])));
          }
        }
        else
        {
          for (int i=0; i<N; ++i) {
            row2_t r2 = out.row(selector(mapping[i],i));
            Functor<row2_t, row1_t>::run(r2, scale, 
                                         in.row(selector(i, mapping[i])));
          }
        }
      }

      template<class Derived1, class Derived2>
      static void uncompress2d(const MatrixBase<Derived1>& in, MatrixBase<Derived2>& out, 
                               const std::vector<int>& mappingRow, const std::vector<int>& mappingCol, double scale)
      {
        typedef typename Eigen::internal::traits<Derived1>::Scalar scalar1_t;
        typedef typename Eigen::internal::traits<Derived2>::Scalar scalar2_t;

        const int Nc = static_cast<int>( selector(in.cols(), out.cols()) );
        const int Nr = static_cast<int>( selector(in.rows(), out.rows()) );
        ocra_assert(Nc == mappingCol.size());
        ocra_assert(Nr == mappingRow.size());
        if (scale==1.)
        {
          for (int i=0; i<Nc; ++i)
          {
            int n = mappingCol[i];
            for (int j=0; j<Nr; ++j) {
              scalar2_t& s2 = out(selector(mappingRow[j], j), selector(n, i));
              Functor<scalar2_t, scalar1_t>::run(s2,
                                                 in(selector(j, mappingRow[j]), selector(i, n)));
            }
          }
        }
        else
        {
          for (int i=0; i<Nc; ++i)
          {
            int n = mappingCol[i];
            for (int j=0; j<Nr; ++j) {
              scalar2_t& s2 = out(selector(mappingRow[j], j), selector(n, i));
              Functor<scalar2_t, scalar1_t>::run(s2, scale,
                                                 in(selector(j, mappingRow[j]), selector(i, n)));
            }
          }
        }
      }
    };

  }//utils
}//ocra

#endif //_OCRA_UTILS_UNCOMPRESS_H_

// cmake:sourcegroup=Utils
