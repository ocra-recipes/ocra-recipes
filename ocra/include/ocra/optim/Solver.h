/** @file Solver.h
  * @brief Declaration file of the Solver class.
  *
  *   Copyright (C) 2009 CEA/DRT/LIST/DTSI/SRCI
  *
  * @author Escande Adrien
  *	@date 09/04/29
  */

#ifndef _OCRABASE_SOLVER_H_
#define _OCRABASE_SOLVER_H_

//ocra includes
#include "ocra/MathTypes.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/MergedVariable.h"
#include "ocra/optim/SolverUtilities.h"
#include "ocra/optim/Objective.h"
#include "ocra/optim/Constraint.h"
#include "ocra/optim/NamedInstance.h"

//boost
#include <boost/circular_buffer.hpp>

//std includes
#include <string>
#include <map>
#include <iosfwd>

/** @namespace ocra
  * @brief Optimization-based Robot Controller namespace. 
  *  a library of classes to write and solve optimization problems dedicated to
  *  the control of multi-body systems. 
  */
namespace ocra
{
  /** @class Solver
    *	@brief %Solver class.
    *	@warning None
    *  
    * This is a base class for every optimization solver in ocra. It provides an interface for derived solvers and some
    * core methods. 
    * It has two responsibilities of its own:
    *   - (i)  keep (lazily) up-do-date the variable of the optimization problem, 
    *   - (ii) maintain the mappings of function's variables relative to this problem variable.
    * (i) can be explain by an example:
    * if we want to optimize the sum of two functions f(x,y) and g(y,z), then we need to solve a problem over 
    * V=(x,y,z). V is said to be the problem variable. If at one point g becomes a function of y only, then we have a 
    * new problem variable V'=(x,y). The solver tracks correctly such changes to keep the problem variable up-to-date.
    * This implies to observe the EVT_CHANGE_DEPENDENCIES events coming from the variables of all constraints and 
    * objectives. This observation is configured in the internalAdd/RemoveObjective/Constraint methods which the user
    * must call when adding/removing an objective/constraint in a derived solver. When this event is triggered, a flag
    * is simply change, the actual recomputation of the problem variable will only take automatically place, if needed, 
    * when the \a solve method is called.
    * (ii) the variable of any constraint or objective in an optimization problem is a subset of the problem variable.
    * Many solvers need to know how a specific variable is mapped with respect to the problem variable. The Solver
    * class provides facilities to keep the mapping of each constraint/objective's variable wrt the problem variable
    * up-to-date. It is done in a transparent way for the user, who just need to use the \a findMapping() method to
    * retrieve the mapping of a variable.
    *
    * Some general rules which must be followed by the derived solver:
    *   - when there are several objectives, they are added in one objective. If not (multi-objective optimization 
    * e.g.), the documentation of the particular solver must report it,
    *   - likewise, when there is several constraints, the problem considers (naturally) their intersection, unless
    * documented differently.
    */
  class Solver
    : public ObserverSubject
    , virtual public NamedInstance
  {
  protected: // restricted access to automap
    const std::vector<int>& findMapping(Variable& var);

    // ------------------------ constructors ------------------------------------
  private:
    /** The class is non-copyable*/
    //@{
    Solver(const Solver&);
    Solver& operator=(const Solver&);
    //@}
  protected:
    /** The class is meant to be derived*/
    Solver();
  public:
    virtual ~Solver() {}

    // ------------------------ public interface --------------------------------
  public:
    /** Translate the actual optimization problem for a specific solver and call its optimization routine.
      * 
      * \internal The following computations are performed:
      *   - update of the problem variable and the resize of the result array if needed,
      *   - call doPrepare(), doSolve() and doConclude()
      *   - set the result as value of the problem variable.
      */
    const OptimizationResult& solve();

    /** Get the last result computed with solve()*/
    const OptimizationResult& getLastResult() const;

    void  printStatus(std::ostream& os) const;

    virtual const std::string& getMoreInfo() const;

    virtual void printValuesAtSolution() = 0;

    /** Returns the state of the solver (e.g. matrices) as a string. */
    virtual std::string toString() const = 0;

    /** Sets the number of solver states that will be kept in memory.
      *
      * If the level is set to 0, the state will never be saved. If it
      * is set to 1, the current state (ie, the state before solving the
      * problem) will be stored when calling solve(). If set to 2, the state
      * before the last two solve() will be stored and so on.
      */
    void setMemoryLevel(int level);

    /** Sets the file where memory will be dumped after solve() errors.
      *
      * Activates auto dumping automatically.
      * If an empty filename is given, throws runtime_error. Else,
      * each time solve() fails, the memorized states (see setMemoryLevel)
      * will be dumped into the specified file from older to newer state.
      */
    void setAutoDumpFile(const std::string& file);
    
    /** Activates auto dumping when solver fails. */
    void activateAutoDump();

    /** Deactivates auto dumping when solver fails. */
    void deactivateAutoDump();

    /** Dumps the memory.
      *
      * Empty string parameter (default): dumps on standard output.
      * Else, dumps in specified file, from older to newer state.
      */
    void dump(const std::string& file = "") const;


    // ------------------------ protected methods -------------------------------
  protected:
    int n() const {return _problemVariable.getVariable().getSize();}

    const Variable& getProblemVariable() const {return _problemVariable.getVariable();}

    void setVariableValue(const VectorXd& value) {_problemVariable.setValue(value);}

    /** \internal Three sub-methods called by solve()
      * They are called one after the other so that writing a command at the end of one has the same effect as writing
      * it at the beginning of the next. However, programmers should respect the following convention when deriving
      * these methods:
      * - doPrepare should manage the memory and perform the translation from the ocra objectives and constraints to the 
      * input data of the optimization routine
      * - doSolve is only meant to call to the solver optimization routine with the data properly ordered and then to
      * fill _result according to the output of the routine.
      * - doConclude perform whatever needs to be done after solving the problem (for example free memory, manage an
      * active set,..)
      */
    //@{
    virtual void doPrepare() = 0;
    virtual void doSolve() = 0;
    virtual void doConclude() = 0;
    //@}


    /** \internal These methods MUST be called whenever a function (objective or constraint) is added to/removed from
      * the optimization problem. Failing to do that will induce an erroneous management of the problem variable and
      * the mappings.
      *
      * \throw These methods will throw a runtime_error if one attempts to add an already added objective/function or
      * to remove one that does not exist in the problem.
      */
    void internalAddObjective(const GenericObjective& objective);
    void internalAddConstraint(const GenericConstraint& constraint);
    void internalRemoveObjective(const GenericObjective& objective);
    void internalRemoveConstraint(const GenericConstraint& constraint);

    /** \internal Callback method to be invoked when the function of a constraint indicates an EVT_RESIZE, or the
      * constraint itself triggers a EVT_CHANGE_BOUNDS_NUMBER event.
      */
    virtual void onConstraintResize(int timestamp);

    /** \internal Callback method to be invoked when the function of an objective triggers a EVT_RESIZE event. */
    virtual void onObjectiveResize(int timestamp);

    // ------------------------ protected members ---------------------------------
  protected:
    OptimizationResult                    _result;  //< structure to store the result of an optimization problem.

    // ------------------------ private members ---------------------------------
  private:
    bool                                  _isVariableUpToDate;  //< a flag on the problem variable validity
    MergedVariable                        _problemVariable;     //< a composite variable built by merging the functions' variables and storing their mappings
    std::vector<const GenericObjective*>  _objectives;          //< set of objectives
    std::vector<const GenericConstraint*> _constraints;         //< set of constraints
    boost::circular_buffer<std::string>   _memory;              //< stores the state of the solver before doSolve if memory level > 0
    std::string                           _autodumpFile;        //< if not empty, file where the saved data will be dumped if an error occurs
    bool                                  _autodump;
  };
}

#endif	//_OCRABASE_SOLVER_H_

// cmake:sourcegroup=Solvers
