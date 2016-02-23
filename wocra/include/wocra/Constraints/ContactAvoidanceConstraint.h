/**
 * \file ContactAvoidanceConstraint.h
 * \author Joseph Salini
 *
 * \brief Define contact avoidance constraint for wOcra controller.
 */

#ifndef __CONTACTAVOIDANCECONSTRAINT_H__
#define __CONTACTAVOIDANCECONSTRAINT_H__


#include "wocra/Constraints/wOcraConstraint.h"

namespace wocra
{

/** \addtogroup constraint
 * \{
 */




/** \brief Create a linear function that represents the joint limit function.
 *
 * The contact avoidance constraint is written at first as: \f$  \vec{0} < \vec{d}_{oa} \f$
 * where \f$ \vec{d}_{oa} \f$ is the contatenation of the minimal distances between the couples of shapes that should not collide.
 *
 * \image html obstacle_avoidance.svg
 *
 * To relate this constraint with the dynamic variables, we constrain the estimated future distances to remain positive, as explained in wocra::JointLimitFunction .
 *
 * In this case, the constraint is expressed as follows:
 *
 * \f{align*}{
 *      \vec{d}_{oa} - \vec{m} &> \vec{d}_{oa}(h) \quad \text{at instant h} \\
 *      \vec{d}_{oa} - \vec{m} &> \dot{\vec{d}}_{oa} h + \ddot{\vec{d}}_{oa} \frac{h^2}{2} \\
 *      \vec{d}_{oa} - \vec{m} &> \dot{\vec{d}}_{oa} h + \left( \J_{oa} \ddq + \dJ_{oa} \dq \right) \frac{h^2}{2}
 * \f}
 *
 * where \f$ \vec{m} \f$ is a margin vector. So it becomes:
 *
 * \f{align*}{
 *      \A \x + \b &> \vec{0}
 *      & &\Leftrightarrow
 *      & \begin{bmatrix} - \J_{oa} \end{bmatrix} \ddq + \begin{bmatrix} - \dJ_{oa} \dq + 2 \left( \vec{d}_{oa} - \vec{m} - \dot{\vec{d}}_{oa} h \right) / h^2  \end{bmatrix}  & > \vec{0}
 * \f}
 *
 * There is some similarity with the wocra::JointLimitFunction because we constrain an estimated future state. The same issues arise (it only constrain the final point, no middle points),
 * so it is interesting to look at the inflexion point. In the same manner, the time of inflexion for a constant acceleration such as the inflexion point is on 0 is computed as follows:
 *
 * \f{align*}{
 *      t_{max} &= 2*(\vec{d}_{oa} - \vec{m}) / \dot{\vec{d}}_{oa} & & \text{(element-wise operations)}
 * \f}
 *
 * For each dof (each line \f$ i \f$ ), we test where is the time of inflexion.
 * If \f$ 0 < t_{max}[i] < h\f$, then the inflexion point is in the horizon of time, we should consider to constrain the motion of this contact avoidance:
 *
 * \f{align*}{
 *      \ddot{\vec{d}}_{max}[i] &= \frac{ \dot{\vec{d}}_{oa}[i]^2 }{ 2( \vec{d}_{oa}[i] - \vec{m}[i] ) }  &  & \Rightarrow & \begin{bmatrix} - \J_{oa} \end{bmatrix} [i] \ddq + \ddot{\vec{d}}_{max}[i] > 0
 * \f}
 *
 *
 * When all these constraints have been defined, we select the tightest ones for each dof.
 *
 */
class ContactAvoidanceFunction: public ocra::LinearFunction
{
    public:
        typedef ocra::LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        ContactAvoidanceFunction(const ocra::Model& m, ocra::Variable& var);
        ~ContactAvoidanceFunction();

        // get/set horizon of prediction
        double getHorizonOfPrediction() const;
        void   setHorizonOfPrediction(double newHpos);
        double getMargin() const;
        void   setMargin(double newMargin);

        void updateContactInformation(const Eigen::MatrixXd& _JObst, const Eigen::VectorXd& _dJdqOst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst);

    private: // Forbid copy
        ContactAvoidanceFunction(ContactAvoidanceFunction&);
        ContactAvoidanceFunction& operator= (const ContactAvoidanceFunction&);

    protected:
        void updateJacobian() const;
        void updateb()        const;
        //    void buildA();

        //    virtual void doUpdateInputSizeBegin();
        //    virtual void doUpdateInputSizeEnd();

        virtual void doUpdateDimensionBegin(int newDimension);
        virtual void doUpdateDimensionEnd(int oldDimension);

    protected:
//        const ocra::Model&   _model;
//        ocra::Variable&      _q_ddot;

        double hpos; // the horizon of prediction for the future joint position;
        double margin;

        Eigen::MatrixXd JObst;
        Eigen::VectorXd dJdqObst;
        Eigen::VectorXd distObst;
        Eigen::VectorXd velObst;

        void computeFullJacobian(const Eigen::MatrixXd& _JObst, Eigen::MatrixXd& fullJacobian) const;
        void computeFullb(const Eigen::VectorXd& _dJdqObst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst, Eigen::VectorXd& fullb) const;

};






/** \brief Create a linear function that represents the contact avoidance function for the full formalism.
 *
 * See \ref wocra::ContactAvoidanceFunction for more information.
 */
class FullContactAvoidanceFunction: public ContactAvoidanceFunction
{
    public:
        typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        FullContactAvoidanceFunction(const ocra::Model& model);
        ~FullContactAvoidanceFunction();

    protected:
        virtual void updateJacobian() const;
        virtual void updateb()        const;

};




/** \brief Create a linear function that represents the contact avoidance function for the reduced formalism.
 *
 * See \ref wocra::ContactAvoidanceFunction for more information.
 */
class ReducedContactAvoidanceFunction: public ContactAvoidanceFunction
{
    public:
        typedef LinearFunction  functionType_t;     //< alias on the type of the mother class. Needed to duplicate the function tree.

        ReducedContactAvoidanceFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation);
        ~ReducedContactAvoidanceFunction();

    protected:
        virtual void updateb()        const;

    private:
        struct Pimpl;
        boost::shared_ptr<Pimpl> pimpl;

        virtual void doUpdateInputSizeBegin();
        virtual void doUpdateInputSizeEnd();
};








class ContactAvoidanceConstraint: public wOcraConstraint
{
public:
    ContactAvoidanceConstraint(const ocra::Model& model, double hpos, double margin);
    virtual ~ContactAvoidanceConstraint() {};

    double getHorizonOfPrediction() const;
    void   setHorizonOfPrediction(double newHpos);

    double getMargin() const;
    void   setMargin(double newMargin);

    void updateContactInformation(const Eigen::MatrixXd& _JObst, const Eigen::VectorXd& _dJdqOst, const Eigen::VectorXd& _distObst, const Eigen::VectorXd& _velObst);

protected:
    virtual void connectToController(const wOcraDynamicFunction& dynamicEquation, bool useReducedProblem);
    virtual void disconnectFromController();


private:
    ContactAvoidanceFunction* createFullContactAvoidanceFunction(const ocra::Model& model);
    ContactAvoidanceFunction* createReducedContactAvoidanceFunction(const ocra::Model& model, const wOcraDynamicFunction& dynamicEquation);

    ContactAvoidanceFunction* _contactAvoidanceFunction;
    const ocra::Model&   _model;
    bool                _is_connected;

    double              _hpos;
    double              _margin;
};


 /** \} */ // end group constraint

}


#endif
