/** @file utilities.h
*   @brief Some utilities function for debbuging purpose.
*
*          Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
*
*   @author Escande Adrien
*   @author Brisset Julien
*	  @date 09.07.17
*/



#ifndef _OCRABASE_UTILITIES_H_
#define _OCRABASE_UTILITIES_H_

#include "ocra/MathTypes.h"

#include <string>
#include <fstream>
#include <iostream>

namespace ocra
{
  namespace utils
  {
    template <class Derived>
    inline void writeInFile(const MatrixBase<Derived>& m, const std::string& fileName, bool app=false)
    {
      std::ofstream aof;
      if (app)
      {
        aof.open(fileName.c_str(), std::ios_base::app);
      }
      else
      {
        aof.open(fileName.c_str());
      }
      if (aof.is_open())
      {
        if (m.cols() != 1)
          aof << m << std::endl;
        else
          aof << m.transpose() << std::endl;
        aof.close();
      }
    }


    template<class Derived>
    inline std::ostream& writeInFile(std::ostream & s, const MatrixBase<Derived>& m, int precision = -1, const std::string& coeffSeparator = " ",
                                     const std::string& rowSeparator = "\n", const std::string& rowPrefix="", const std::string& rowSuffix="",
                                     const std::string& matPrefix="", const std::string& matSuffix="")
    {
      std::string rowSpacer = "";
      int i = int(matSuffix.length())-1;
      while (i>=0 && matSuffix[i]!='\n')
      {
        rowSpacer += ' ';
        i--;
      }

      std::streamsize old_precision = 0;
      if(precision > 0) old_precision = s.precision(precision);
      s << matPrefix;
      for(int i = 0; i < (int)m.get_nrows(); ++i)
      {
        if (i)
          s << rowSpacer;
        s << rowPrefix;
        s << m(i, 0);
        for(int j = 1; j < (int)m.get_ncols(); ++j)
        {
          s << coeffSeparator;
          s << m(i, j);
        }
        s << rowSuffix;
        if( i <(int) m.get_nrows() - 1)
          s << rowSeparator;
      }
      s << matSuffix;
      if(precision>0) s.precision(old_precision);
      return s;
    }


    /** Overload for a vector*/
    template<class Derived>
    inline std::ostream& writeInFile(std::ostream & s, const MatrixBase<Derived>& v, int precision = -1, const std::string& coeffSeparator = " ",
                                     const std::string& vectPrefix="", const std::string& vectSuffix="")
    {
      assert(v.rows() == 1 || v.cols() == 1);
      std::streamsize old_precision = 0;
      if(precision > 0) old_precision = s.precision(precision);
      s << vectPrefix;
      s << v[0];
      for(int j = 1; j < (int)v.getSize(); ++j)
      {
        s << coeffSeparator;
        s << v[j];
      }
      s << vectSuffix;
      if(precision>0) s.precision(old_precision);
      return s;
    }

    template<class Derived>
    inline std::ostream& writeInFileForScilab(std::ostream & s, const MatrixBase<Derived>& m, int precision = -1)
    {
      if (m.rows() == 1 || m.cols() == 1)
        return writeInFile(s, m, precision, ", ", "[", "];");
      else
        return writeInFile(s, m, precision, ", ", ";\n", "[", "]", "[", "];");
    }
  }





  class Variable;
  class QuadraticSolver;

  struct ToScilab
  {
    template<typename Derived>
    ToScilab(const Eigen::MatrixBase<Derived>& m)
    {
      std::string lineSep = ";";
      lineSep+= (m.cols()==1)?"":"\n";
      Eigen::IOFormat f(precision, 0, ", ", lineSep, "", "", "[", "]");
      std::ostringstream os;
      os << m.format(f);
      str = os.str();
    }

    ToScilab(const Variable& v);
    ToScilab(const QuadraticSolver& s);

    static int precision;

  private:
    std::string str;

    friend std::ostream& operator<< (std::ostream& os, const ToScilab& m);
  };

  std::ostream& operator<< (std::ostream& os, const ToScilab& obj);

}

#endif

// cmake:sourcegroup=Utils

