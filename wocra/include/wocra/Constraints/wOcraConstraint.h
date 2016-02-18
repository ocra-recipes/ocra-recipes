/**
 * \file wOcraConstraint.h
 * \author Joseph Salini
 *
 * \brief Define base class that can be used as constraints in wOcra controller.
 */

#ifndef __wOcraCONSTRAINT_H__
#define __wOcraCONSTRAINT_H__


#include "ocra/optim/LinearFunction.h"
#include "ocra/control/Model.h"
#include "ocra/control/FullDynamicEquationFunction.h"
#include "ocra/optim/Variable.h"

#include "ocra/optim/Constraint.h"


namespace wocra
{

/** \addtogroup constraint
 * \{
 */

class wOcraConstraint
{
public:
    wOcraConstraint()
        : _constraint() {};

    virtual ~wOcraConstraint() {};

    ocra::LinearConstraint& getConstraint()
    {
        return *_constraint.get();
    }

protected:
    friend class wOcraController;    //Only the wOcraController should know about the following functions
    virtual void connectToController(const ocra::FullDynamicEquationFunction& dynamicEquation, bool useReducedProblem) = 0;
    virtual void disconnectFromController() = 0;

    boost::shared_ptr< ocra::Constraint<ocra::LinearFunction> >   _constraint;
};




/** \} */ // end group constraint

}



#endif
