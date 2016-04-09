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

namespace ocra
{
    typedef std::shared_ptr<HierarchyLevel> HierarchyLevelPtr;
    typedef std::shared_ptr<HierarchyLevel_barre> HierarchyLevel_barrePtr;
    typedef std::shared_ptr<EqualitiesConstraints> EqualitiesConstraintsPtr;
    typedef std::shared_ptr<InequalitiesConstraints> InequalitiesConstraintsPtr;
    typedef std::shared_ptr<Solution> SolutionPtr;
    typedef std::shared_ptr<MatrixPQ> MatrixPQPtr;
}

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class CascadeQP
    *	@brief %CascadeQP class.
    *	@warning Solution is bound by arbitrary big bounds (1e10). If your problem scale
    * has this order of magnitude, consider changing this bounds
    *  
    * This class implements a hierarchical QP Solver based on
    * "Prioritizing linear equality and inequality systems: application to local motion
    * planning for redundant robots"
    * O. Kanoun, F. Lamiraux, P.-B. Wieber, F. Kanehiro, E. Yoshida, J.-P. Laumond
    * ICRA 2009
    */
  class CascadeQP
  {

  public:
    CascadeQP();
    virtual ~CascadeQP();

  public:
    std::size_t addHierarchyLevel(ocra::HierarchyLevelPtr h);
    std::size_t addHierarchyLevel(const std::vector<HierarchyLevelPtr>& v);

    void  clear(void);
    const FinalSolution& solveCascadeQP();

    const FinalSolution& getSolution() const;
    const Eigen::VectorXd& getSolutionOfLevel(int i) const;
    const HierarchyLevel& getHierarchyLevel(int i) const;
    std::size_t getNumberOfHierarchyLevel() const;
  protected:
    void addHierarchyLevel_barre(int i);

    std::size_t initializeHierarchyLevel_barre();

    void computeMatrixPQ(int i);
    void computeEqualitiesConstraints(int i);
    void computeInequalitiesConstraints(int i);
    void computeHierarchyLevel_barre(int i);

  protected:
    std::size_t taille;
   
    std::vector<SolutionPtr> allSolution;
    std::vector<HierarchyLevelPtr> allHierarchyLevel;
    std::vector<HierarchyLevel_barrePtr> allHierarchyLevel_barre;
    std::vector<MatrixPQPtr> allMatrixPQ;
    std::vector<EqualitiesConstraintsPtr> allEqualitiesConstraints;
    std::vector<InequalitiesConstraintsPtr> allInequalitiesConstraints;
    std::vector<int> indexConstraintsViolated ;
    std::vector<int> indexConstraintsNotViolated ;
    FinalSolution f;
    
  };
}

#endif	//_OCRABASE_CASCADE_QP_H_
