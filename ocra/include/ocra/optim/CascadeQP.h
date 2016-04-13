/** @file CascadeQP.h
  * @brief Declaration file of the CascadeQP class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Yacoubi Salim
  * @author Escande Adrien
  *	@date 09/07/24
  */

#ifndef _OCRABASE_CASCADE_QP_H_
#define _OCRABASE_CASCADE_QP_H_

#include "ocra/optim/CascadeQPStructures.h"
#include <memory>
#include <vector>

//#ifdef USE_QPOASES
#include <qpOASES.hpp>
//#endif
/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace.
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems.
  */
namespace ocra
{
/** @class CascadeQP
  * @brief %CascadeQP class.
  */
class CascadeQP
{
    DEFINE_CLASS_POINTER_TYPEDEFS(CascadeQP)
public:
    CascadeQP();
    virtual ~CascadeQP();

public:
    std::size_t addHierarchyLevel(ocra::HierarchyLevel::Ptr h);
    std::size_t addHierarchyLevel(const std::vector<HierarchyLevel::Ptr>& v);

    void  clear(void);
    const FinalSolution& solveCascadeQP();

    const FinalSolution& getSolution() const;
    const Eigen::VectorXd& getSolutionOfLevel(int i) const;
    HierarchyLevel::Ptr getHierarchyLevel(int i) const;
    std::size_t getNumberOfHierarchyLevel() const;
protected:
    void addHierarchyLevel_barre(int i);

    std::size_t initializeHierarchyLevel_barre();

    void computeMatrixPQ(int i);
    void computeEqualitiesConstraints(int i);
    void computeInequalitiesConstraints(int i);
    void computeHierarchyLevel_barre(int i);

protected:
    std::vector<Solution::Ptr> allSolution;
    std::vector<HierarchyLevel::Ptr> allHierarchyLevel;
    std::vector<HierarchyLevel_barre::Ptr> allHierarchyLevel_barre;
    std::vector<MatrixPQ::Ptr> allMatrixPQ;
    std::vector<EqualitiesConstraints::Ptr> allEqualitiesConstraints;
    std::vector<InequalitiesConstraints::Ptr> allInequalitiesConstraints;
    std::vector<int> indexConstraintsViolated ;
    std::vector<int> indexConstraintsNotViolated ;
    FinalSolution f;

};
}

#endif	//_OCRABASE_CASCADE_QP_H_
