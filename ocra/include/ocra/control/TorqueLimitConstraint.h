/**
 * \file TorqueLimitConstraint.h
 * \author Joseph Salini
 *
 * \brief Define torque limit constraint for wOcra controller.
 */

#ifndef __TORQUELIMITCONSTRAINT_H__
#define __TORQUELIMITCONSTRAINT_H__


#include "ocra/optim/LinearFunction.h"
#include "ocra/control/Model.h"
#include "ocra/optim/Constraint.h"


namespace ocra
{

/** \addtogroup constraint
 * \{
 */



/** \brief Create a linear function that represents the torque limit function.
 *
 * The torque limit inequality is:
 * \f[
 *      - \torque_{max} < \torque < \torque_{max}
 * \f]
 *
 * It returns an equation of the form:
 *
 * \f{align*}{
 *      \A \x + \b &> \vec{0} & &\Leftrightarrow & \begin{bmatrix} - \Id{} \\ \Id{} \end{bmatrix} .  \torque  + \begin{bmatrix} \torque_{max} \\ \torque_{max} \end{bmatrix} &> \vec{0}
 * \f}
 */
class TorqueLimitFunction: public LinearFunction
{
public:
    typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

    TorqueLimitFunction(const Model& model);
    ~TorqueLimitFunction();

    void  setTorqueLimits(const Eigen::VectorXd& torqueLimits);
    const Eigen::VectorXd& getTorqueLimits() const;

protected:
    void updateJacobian() const;
    void updateb()        const;

    Eigen::VectorXd       _torqueLimits;
    int                   _torqueDim;

private: // Forbid copy
    TorqueLimitFunction(TorqueLimitFunction&);
    TorqueLimitFunction& operator= (const TorqueLimitFunction&);
};






class TorqueLimitConstraint: public LinearConstraint
{
public:
    TorqueLimitConstraint(const Model& model)
        : LinearConstraint(createTorqueLimiFunction(model)) {}; //GREATER_THAN_ZERO

    TorqueLimitConstraint(const Model& model, const Eigen::VectorXd& torqueLimits)
        : LinearConstraint(createTorqueLimiFunction(model)) //GREATER_THAN_ZERO
    {
        _torqueLimitFunction->setTorqueLimits(torqueLimits);
    };

    virtual ~TorqueLimitConstraint()
    {
        delete _torqueLimitFunction;
    }

    void  setTorqueLimits(const Eigen::VectorXd& torqueLimits) {_torqueLimitFunction->setTorqueLimits(torqueLimits);};
    const Eigen::VectorXd& getTorqueLimits() const {return _torqueLimitFunction->getTorqueLimits();};

private:
    TorqueLimitFunction* createTorqueLimiFunction(const Model& model)
    {
        _torqueLimitFunction = new TorqueLimitFunction(model);
        return _torqueLimitFunction;
    }

    TorqueLimitFunction* _torqueLimitFunction;
};

/** \} */ // end group constraint

}


#endif
