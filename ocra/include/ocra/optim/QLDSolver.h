/** @file QLDSolver.h
  * @brief Declaration file of the QLDSolver class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/06/11
  */

#ifndef _OCRABASE_QLD_SOLVER_H_
#define _OCRABASE_QLD_SOLVER_H_

//ocra includes
#include "ocra/optim/QuadraticSolver.h"
#include "ocra/optim/ObjQLD.h"
#include "ocra/optim/Buffer.h"

//std includes
#include <vector>

using Eigen::Map;

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class QLDSolver
    *	@brief %QLDSolver class.
    *	@warning None
    *  
    * Wrapping of the QP solver from QLD
    * 
    */
  class QLDSolver : public QuadraticSolver
  {
    // ------------------------ structures --------------------------------------
  public:
    typedef Map<MatrixXd> MatrixMap;
    typedef Map<VectorXd> VectorMap;

    // ------------------------ constructors ------------------------------------
  private:
    QLDSolver(QLDSolver&);
  protected:
  public:
    QLDSolver();

    // ------------------------ public interface --------------------------------
  public:
    void setTolerance(double epsilon);

    double getTolerance(void) const;

    const std::string& getMoreInfo() const;

    MatrixXd getP() const;
    VectorXd getq() const;
    MatrixXd getA() const;
    VectorXd getb() const;
    VectorXd getbp() const;
    MatrixXd getC() const;
    VectorXd getd() const;
    VectorXd getl() const;
    VectorXd getu() const;
    VectorXd getxl() const;
    VectorXd getxu() const;

    // ------------------------ protected methods -------------------------------
  protected:
    void doPrepare();
    void doSolve();
    void doConclude();

    // ------------------------ private methods ---------------------------------
  private:
    void resize();

    void updateObjectiveEquations();
    void updateEqualityEquations();
    void updateInequalityEquations();
    void updateBounds();

    //unsigned int transferSimpleInequalityToBounds(MatrixBase& C, VectorBase& d);      //return the number of 'suppressed' row in C
    //bool isASingleComponentRow(const MatrixBase& C, const int rowIndex, int& returnColIndex);

    void translateReturnInfo(eReturnInfo& orcInfo, int qldInfo);

    // ------------------------ private members ---------------------------------
  private:
    ObjQLD*  _solver;

    MatrixMap _C;   //< P (nxn)
    VectorMap _d;   //< q (n)
    MatrixMap _A;   //< A (mxn)
                    //< C (pxn)
    VectorMap _b;   //< b (m)
                    //< d (p)
    VectorMap _xl;
    VectorMap _xu;
    double    _infinity;
    double    _eps;

    int info;     //< to store the return info of the qld

    size_t _pt;   //< number of unidimensional inequality constraints transfered to bounds
    
    Buffer<double>  _buffer;
  };

  void testQLDSolver();
}

#endif	//_OCRABASE_QLD_SOLVER_H_

// cmake:sourcegroup=Solvers
