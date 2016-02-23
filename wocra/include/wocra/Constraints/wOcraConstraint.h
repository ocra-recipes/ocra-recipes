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
#include "ocra/optim/Variable.h"

#include "ocra/optim/Constraint.h"


namespace wocra
{

/** \addtogroup constraint
 * \{
 */


///////// FOR DYNAMIC EQUATION FUNCTION!!!
/** \brief Create a linear function that represents the dynamic equation of motion.
 *
 * The equation of motion is:
 * \f[
 *      \M \ddq + \n + \g = S \torque - \J_c\tp \force_c
 * \f]
 *
 * So given the variable of our problem \f$ \x = \begin{bmatrix}  \ddq\tp & \torque\tp & \force_c\tp\end{bmatrix}\tp \f$ It returns an equation of the form:
 *
 * \f{align*}{
 *      \A \x + \b &= \vec{0} & &\Leftrightarrow & \begin{bmatrix} \M  &  -S  & \J_c\tp \end{bmatrix} . \begin{bmatrix}  \ddq \\ \torque \\ \force_c\end{bmatrix} + [ \n + \g ] = \vec{0}
 * \f}
 */
class wOcraDynamicFunction: public ocra::LinearFunction
{
    public:
        typedef ocra::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        wOcraDynamicFunction(const ocra::Model& model);
        ~wOcraDynamicFunction();

        void takeIntoAccountGravity(bool useGrav);

        const Eigen::MatrixXd& getInertiaMatrixInverseJchiT() const;
        const Eigen::VectorXd& getInertiaMatrixInverseLinNonLinGrav() const;
        ocra::Variable&         getActionVariable() const;

    protected:
        void updateJacobian() const;
        void updateb()        const;
        void buildA();

        virtual void doUpdateInputSizeBegin();
        virtual void doUpdateInputSizeEnd();


    private: // Forbid copy
        wOcraDynamicFunction(wOcraDynamicFunction&);
        wOcraDynamicFunction& operator= (const wOcraDynamicFunction&);

        static ocra::Variable& createDEVariable(const ocra::Model& model);

        void invalidateReducedProblemData(int timestamp);

    private:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;

};





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
    virtual void connectToController(const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem) = 0;
    virtual void disconnectFromController() = 0;

    boost::shared_ptr< ocra::Constraint<ocra::LinearFunction> >   _constraint;
};




/** \} */ // end group constraint

}



#endif
