#include "ocra/optim/WeightedSquareDistanceFunction.h"
#include <stdexcept>

namespace ocra
{
  void WeightedSquareDistanceFunction::doChangePi(const MatrixXd& Pi, int index)
  {
    ocra_assert(Pi.cols() == x.getSize());
    ocra_assert(Pi.rows() == x.getSize());

    //check if Pi is a diagonal matrix
    if (!Pi.isDiagonal())
      throw std::runtime_error("[WeightedSquareDistanceFunction::changePi] Given Pi is not a diagonal matrix.");

    _weight = Pi.diagonal();

    computeP();
    computeq();
    computer();
  }


  void WeightedSquareDistanceFunction::doChangeqi(const VectorXd& qi, int index)
  {
    throw std::runtime_error("[WeightedSquareDistanceFunction::changeqi] changeqi is not available directly. Please use changeWeight and changeReference instead.");
  }


  void WeightedSquareDistanceFunction::doChangeri(double ri, int index)
  {
    throw std::runtime_error("[WeightedSquareDistanceFunction::changeqi] changeri is not available directly. Please use changeWeight and changeReference instead.");
  }

  void WeightedSquareDistanceFunction::changeWeight(double weight)
  {
    _defaultWeight = weight;
    _weight.fill(_defaultWeight);
    computeP();
    computeq();
    computer();
    if (weight>0)
      changeConvexityProperty(CONVEXITY_STRICTLY_CONVEX);
    else if (weight<0)
      changeConvexityProperty(CONVEXITY_STRICTLY_CONCAVE);
    else
      changeConvexityProperty(CONVEXITY_CONVEX_AND_CONCAVE);

    invalidateAll();
    propagate<EVT_CHANGE_VALUE>();
  }


  void WeightedSquareDistanceFunction::updateValue() const
  {
    _value[0] = (_weight.array() * (x.getValue().array()-_reference.array()).square()).sum();
  }


  void WeightedSquareDistanceFunction::updateJacobian() const
  {
    _jacobian.row(0).array() = 2*_weight.array() * x.getValue().array() + getqi(0).array();
  }


  void WeightedSquareDistanceFunction::doUpdateInputSizeBegin()
  {
    // Do nothing. This overload is just here to enable input resizing (default implementation throw an exception).
  }


  void WeightedSquareDistanceFunction::doUpdateInputSizeEnd()
  {
    int s = static_cast<int>(_weight.size());

    if (x.getSize()>s)
    {
      //we need to fill the end of _weight and _reference with default values
      _weight.conservativeResize(x.getSize());
      _weight.tail(x.getSize()-s).setConstant(_defaultWeight);

      _reference.conservativeResize(x.getSize());
      _reference.tail(x.getSize()-s).setZero();
    }
    else
    {
      _weight.conservativeResize(x.getSize());
      _reference.conservativeResize(x.getSize());
    }

    QuadraticFunction::doUpdateInputSizeEnd();
    computeP();
    computeq();
    computer();
  }


  void WeightedSquareDistanceFunction::computeP()
  {
    IFunction<PARTIAL_XX>::_val[0]->setZero();
    IFunction<PARTIAL_XX>::_val[0]->diagonal() = _weight;
  }

  void WeightedSquareDistanceFunction::computeq()
  {
    _q[0]->array() = - _weight.array()*_reference.array();
  }

  void WeightedSquareDistanceFunction::computer()
  {
    _r[0] = 0.5*(_reference.array()*_weight.array()*_reference.array()).sum();
  }

}


#include "ocra/optim/Variable.h"

namespace ocra
{
  void testWeightedSquareDistanceFunction(void)
  {
    BaseVariable x("x", 3);
    BaseVariable y("y", 4);
    BaseVariable z("z", 5);

    CompositeVariable X("X");
    X.add(x).add(y);

    VectorXd ref(X.getSize());
    ref.setZero();


    WeightedSquareDistanceFunction f(X, 1., ref);

    VectorXd v(X.getSize());
    v.setConstant(1);

    X.setValue(v);
    std::cout << f.getValue() << std::endl;
    std::cout << std::endl;

    X.add(z);
    v.resize(X.getSize());
    v.setConstant(1);

    X.setValue(v);
    std::cout << f.getValue() << std::endl;
    std::cout << std::endl;

    X.remove(y);
    v.resize(X.getSize());
    v.setConstant(1);

    X.setValue(v);
    std::cout << f.getValue() << std::endl;
    std::cout << std::endl;

    VectorXd w(X.getSize());
    for (int i=0; i<X.getSize(); ++i)
      w[i] = i+1;
    f.changeWeight(w);

    std::cout << f.getValue() << std::endl;
    std::cout << std::endl;
  }
}

// cmake:sourcegroup=Function

