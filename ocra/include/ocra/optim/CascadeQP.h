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

// includes
#include "ocra/optim/CascadeQPStructures.h"
#include "ocra/optim/ObjQLD.h"

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
    * Base solver used for a QP problem is QLD.
    */
  class CascadeQP
  {
    // ------------------------ structures --------------------------------------
  public:
  protected:
  private:

    // ------------------------ public static members ---------------------------
  public:

    // ------------------------ constructors ------------------------------------
  private:
  protected:
  public:
    // Constructeur :
    CascadeQP();

    // Destructeur :
    ~CascadeQP();

    // ------------------------ public interface --------------------------------
  public:
    // Add Hierarchy Level :
    size_t addHierarchyLevel(HierarchyLevel *h);
    size_t addHierarchyLevel(const std::vector<HierarchyLevel *>& v);

    //solve
    void  clear(void);  //clear the actual hierarchy problem
    const FinalSolution& solveCascadeQP();

    //getter
    const FinalSolution& getSolution() const;
    const Vector& getSolutionOfLevel(int i) const;
    const HierarchyLevel& getHierarchyLevel(int i) const;
    size_t getNumberOfHierarchyLevel() const;
    //const allHierarchyLevel& getAllHierarchyLevel() const;

    // ------------------------ public methods ----------------------------------
  public:

    // ------------------------ public static methods ---------------------------
  public:

    // ------------------------ protected methods -------------------------------
  protected:
    // Add Hierarchy Level_barre :
    void addHierarchyLevel_barre(int i);

    // Initialize Equalities & Inequalities Constraints :
    size_t initializeHierarchyLevel_barre();

    // Compute :
    void computeMatrixPQ(int i);
    void computeEqualitiesConstraints(int i);
    void computeInequalitiesConstraints(int i);
    void computeHierarchyLevel_barre(int i);

    // Solve :
    int solveQLD(int i);

    // ------------------------ protected static methods ------------------------
  protected:

    // ------------------------ private methods ---------------------------------
  private:

    // ------------------------ private static methods --------------------------
  private:

    // ------------------------ protected members -------------------------------
  protected:
    // taille des matrices 
    cfl_size_t taille;      //todo [mineur] still in use ?
   
    // Vector
    std::vector<Solution *> allSolution;
    std::vector<HierarchyLevel *> allHierarchyLevel;
    std::vector<HierarchyLevel_barre *> allHierarchyLevel_barre;
    std::vector<MatrixPQ *> allMatrixPQ;
    std::vector<EqualitiesConstraints *> allEqualitiesConstraints;
    std::vector<InequalitiesConstraints *> allInequalitiesConstraints;
    std::vector<int> indexConstraintsViolated ;
    std::vector<int> indexConstraintsNotViolated ;
    FinalSolution f;

    //solver
    ocra::ObjQLD solver;

    // ------------------------ protected static members ------------------------
  protected:

    // ------------------------ private members ---------------------------------
  private:

    // ------------------------ private static members --------------------------
  private:

    // ------------------------ friendship declarations -------------------------
  };
}

#endif	//_OCRABASE_CASCADE_QP_H_

// cmake:sourcegroup=toBeUpdated

