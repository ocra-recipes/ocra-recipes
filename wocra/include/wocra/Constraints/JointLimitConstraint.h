/**
 * \file JointLimitConstraint.h
 * \author Joseph Salini
 *
 * \brief Define joint limit constraint for wOcra controller.
 */

#ifndef __JOINTLIMITCONSTRAINT_H__
#define __JOINTLIMITCONSTRAINT_H__


#include "wocra/Constraints/wOcraConstraint.h"


namespace wocra
{

/** \addtogroup constraint
 * \{
 */



/** \brief Create a linear function that represents the joint limit function.
 *
 * The joint limit function \f$ \q_{min} < \q < \q{max} \f$ defined in this controller is composed of a couple of inequalities:
 *
 *      - the first one is computed on a horizon h. The system brakes far from the limit, but can pass inside the constraint when close to it (because it only constrains the final point, not the middle ones).
 *      - the second one is computed with the time of inflexion. The system brakes when close to the limits, but some singularities appears when it is on the limits.
 *
 * The two computation have their drawbacks, but together they allow limits avoidance properly.
 *
 * \image html joint_limit_without_inflexion.svg "Joint limit constraint if inflexion point is not taken into account"
 *
 * The first one which constrains the last point over the horizon h can be expressed as follows (depending on \f$ \ddq \f$):
 *
 * \f{align*}{
 *      \q_{min} < \q + \dq h + \ddq \frac{h^2}{2} < \q_{max}
 * \f}
 *
 * \f{align*}{
 *      A \x + \b &> \vec{0}
 *      & &\Leftrightarrow
 *      & \begin{bmatrix} - \Id{} \\ \Id{} \end{bmatrix} \ddq + 2 \begin{bmatrix} \q_{max} - \q + \dq h \\ \q_{min} - \q + \dq h \end{bmatrix} / h^2 &> \vec{0}
 * \f}
 *
 * The second one is more "tricky". First it computes the times of inflexion for a constant acceleration such as the inflexion point is on 0:
 *
 * \f{align*}{
 *      t_{max} &= - 2 ( \q - \q_{max} ) / \dq & &\text{(element-wise operations)} \\
 *      t_{min} &= - 2 ( \q - \q_{min} ) / \dq
 * \f}
 *
 * For each dof (each line \f$ i \f$ ), we test where is the time of inflexion.
 * If \f$ 0 < t_{max}[i] < h\f$, then the inflexion point is in the horizon of time, we should consider to constrain the motion of this dof:
 *
 * \f{align*}{
 *      \ddq_{max}[i] &= \frac{ \dq[i]^2 }{ 2(\q[i] - \q_{max}[i] ) }  &  & \Rightarrow & \ddq [i] + \ddq_{max}[i] > 0
 * \f}
 *
 * Again, if \f$ 0 < t_{min}[i] < h \f$ :
 *
 * \f{align*}{
 *      \ddq_{min}[i] &= \frac{ \dq[i]^2 }{ 2(\q[i] - \q_{min}[i] ) }  &  & \Rightarrow & \ddq [i] + \ddq_{min}[i] > 0
 * \f}
 *
 * When all these constraints have been defined, we select the tightest ones for each dof.
 *
 * \image html joint_limit_inflexion_explaination.svg "Explaination of the inflexion point in the joint limit constraint"
 *
 */
class JointLimitFunction: public ocra::LinearFunction
{
    public:
        typedef ocra::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        JointLimitFunction(const ocra::Model& m, ocra::Variable& var);
        ~JointLimitFunction();

        // get/set horizon of prediction
        double getHorizonOfPrediction() const;
        void   setHorizonOfPrediction(double newHpos);

        void  setJointLimits(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits);
        void  setJointLowerLimits(const Eigen::VectorXd& newLowerLimits);
        void  setJointUpperLimits(const Eigen::VectorXd& newUpperLimits);

        const Eigen::VectorXd& getJointLowerLimits() const;
        const Eigen::VectorXd& getJointUpperLimits() const;

        void  setJointLimit(int i, double newLowerLimit, double newUpperLimit);
        void  setJointLowerLimit(int i, double newLowerLimit);
        void  setJointUpperLimit(int i, double newUpperLimit);

        double getJointLowerLimit(int i) const;
        double getJointUpperLimit(int i) const;

    private: // Forbid copy
        JointLimitFunction(JointLimitFunction&);
        JointLimitFunction& operator= (const JointLimitFunction&);

    protected:
        Eigen::VectorXd     jointLowerLimits;
        Eigen::VectorXd     jointUpperLimits;
        double              hpos; // the horizon of prediction for the future joint position;

        void                computeFullJacobian(int varDofs, int nIDofs, Eigen::MatrixXd& fullJacobian) const;
        void                computeFullb(const Eigen::VectorXd& gpos, const Eigen::VectorXd& gvel, Eigen::VectorXd& fullb) const;
};






/** \brief Create a linear function that represents the joint limit function for the full formalism.
 *
 * See \ref wocra::JointLimitFunction ofr more information.
 */
class FullJointLimitFunction: public JointLimitFunction
{
    public:
        typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        FullJointLimitFunction(const ocra::Model& model);
        ~FullJointLimitFunction();

    protected:
        virtual void updateb() const;

    private:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;
};





/** \brief Create a linear function that represents the joint limit function for the reduced formalism.
 *
 * See \ref wocra::JointLimitFunction ofr more information.
 */
class ReducedJointLimitFunction: public JointLimitFunction
{
    public:
        typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        ReducedJointLimitFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation);
        ~ReducedJointLimitFunction();

    protected:
        virtual void updateJacobian() const;
        virtual void updateb()        const;

    private:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;

        virtual void doUpdateInputSizeBegin();
        virtual void doUpdateInputSizeEnd();
};













class JointLimitConstraint: public wOcraConstraint
{
public:
    JointLimitConstraint(const ocra::Model& model, double hpos=.2);
    JointLimitConstraint(const ocra::Model& model, const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits, double hpos=.2);
    virtual ~JointLimitConstraint() {};

    double getHorizonOfPrediction() const;
    void   setHorizonOfPrediction(double newHpos);

    void  setJointLimits(const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits);
    void  setJointLowerLimits(const Eigen::VectorXd& newLowerLimits);
    void  setJointUpperLimits(const Eigen::VectorXd& newUpperLimits);

    const Eigen::VectorXd& getJointLowerLimits() const;
    const Eigen::VectorXd& getJointUpperLimits() const;

    void  setJointLimit(int i, double newLowerLimit, double newUpperLimit);
    void  setJointLowerLimit(int i, double newLowerLimit);
    void  setJointUpperLimit(int i, double newUpperLimit);

    double getJointLowerLimit(int i) const;
    double getJointUpperLimit(int i) const;

protected:
    virtual void connectToController(const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem);
    virtual void disconnectFromController();


private:
    JointLimitFunction* createFullJointLimitFunction(const ocra::Model& model);
    JointLimitFunction* createReducedJointLimitFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation);

    JointLimitFunction* _jointLimitFunction;
    const ocra::Model&   _model;
    bool                _is_connected;

    double              _hpos;
    Eigen::VectorXd     _lowerLimits;
    Eigen::VectorXd     _upperLimits;
};








 /** \} */ // end group constraint

}


#endif
