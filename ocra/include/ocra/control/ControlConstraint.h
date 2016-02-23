/**
 * \file ControlConstraint.h
 * \author Joseph Salini
 *
 * \brief Define base class that can be used as constraints in wOcra controller.
 */

#ifndef _CONTROL_CONSTRAINT_H_
#define _CONTROL_CONSTRAINT_H_


#include "ocra/optim/LinearFunction.h"
#include "ocra/control/Model.h"
#include "ocra/control/FullDynamicEquationFunction.h"
#include "ocra/optim/Variable.h"
#include "ocra/optim/Constraint.h"


namespace ocra
{

/** \addtogroup constraint
 * \{
 */

class ControlConstraint
{
public:
    ControlConstraint()
        : _constraint() {};

    virtual ~ControlConstraint() {};

    LinearConstraint& getConstraint()
    {
        return *_constraint.get();
    }

    friend class Controller;
    virtual void connectToController(const FullDynamicEquationFunction& dynamicEquation, bool useReducedProblem) = 0;
    virtual void disconnectFromController() = 0;

protected:
    boost::shared_ptr< Constraint<LinearFunction> >   _constraint;
};




/** \} */ // end group constraint

}



#endif
