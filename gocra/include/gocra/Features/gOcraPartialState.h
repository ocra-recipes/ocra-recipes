/**
 * \file gOcraPartialState.h
 * \author Joseph Salini
 *
 * \brief Define partial state classes that can be used to control some joints of the robot.
 *
 * The xde framework propose a full state control. However, sometimes we want to control only a subset of joint,
 * for instance the back joints to stay straight, the neck joints, the arms joint to be in a particular position.
 * That's why we extend the xde framework with these classes.
 */

#ifndef __GOCRAPARTIALSTATE_H__
#define __GOCRAPARTIALSTATE_H__


#include "ocra/optim/NamedInstance.h"
#include "ocra/control/Model.h"
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

using namespace ocra;


namespace gocra
{

/** \addtogroup feature
 * \{
 */

/** \brief A abstract partial state.
 *
 * This class is greatly inspired from the \b FullState class defined in the xde framework.
 */
class PartialState : public NamedInstance
{
public:
    PartialState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart);
    virtual ~PartialState() = 0;

    const Model& getModel() const;
    int getSize() const;
    const Eigen::MatrixXd& getJacobian() const;

    virtual const Eigen::MatrixXd& getInertiaMatrix() const = 0;
    virtual const Eigen::MatrixXd& getInertiaMatrixInverse() const = 0;
    virtual const Eigen::VectorXd& q() const = 0;
    virtual const Eigen::VectorXd& qdot() const = 0;
    virtual const Eigen::VectorXd& qddot() const = 0;
    virtual const Eigen::VectorXd& tau() const = 0;

protected:
    Eigen::VectorXi& getDofs() const;

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};



/** \brief A partial state of the model.
 *
 * It get information about the partial state from a \b Model.
 */
class PartialModelState: public PartialState
{
public:
    PartialModelState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart);
    virtual ~PartialModelState();

    virtual const Eigen::MatrixXd& getInertiaMatrix() const;
    virtual const Eigen::MatrixXd& getInertiaMatrixInverse() const;
    virtual const Eigen::VectorXd& q() const;
    virtual const Eigen::VectorXd& qdot() const;
    virtual const Eigen::VectorXd& qddot() const;
    virtual const Eigen::VectorXd& tau() const;

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};



/** \brief A target for a model partial state.
 *
 * It represents a desired partial state.
 */
class PartialTargetState: public PartialState
{
public:
    PartialTargetState(const std::string& name, const Model& model, const Eigen::VectorXi& selectedDofs, int whichPart);
    virtual ~PartialTargetState();

    virtual const Eigen::MatrixXd& getInertiaMatrix() const;
    virtual const Eigen::MatrixXd& getInertiaMatrixInverse() const;
    virtual const Eigen::VectorXd& q() const;
    virtual const Eigen::VectorXd& qdot() const;
    virtual const Eigen::VectorXd& qddot() const;
    virtual const Eigen::VectorXd& tau() const;

    void set_q(const Eigen::VectorXd&     q);
    void set_qdot(const Eigen::VectorXd&  qdot);
    void set_qddot(const Eigen::VectorXd& qddot);
    void set_tau(const Eigen::VectorXd&   tau);

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl;
};

/** \} */ // end group feature

} //end of namespace gocra


#endif
