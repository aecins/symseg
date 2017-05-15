// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REFINEMENT_BASE_FUNCTOR_HPP
#define REFINEMENT_BASE_FUNCTOR_HPP

// Eigen
#include "unsupported/Eigen/NonLinearOptimization"

namespace sym
{  
  /** \brief Base functor for non-linear optimization with Eigen. All the models
   * that need non linear optimization must define their own one and implement
   * either of:
   *   operator() (const Eigen::VectorXd& x, Eigen::VectorXd& fvec)
   *   operator() (const Eigen::VectorXf& x, Eigen::VectorXf& fvec)
   * dependening on the choosen _Scalar
   */
  template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
  struct BaseFunctor
  {
    typedef _Scalar Scalar;
    
    enum
    {
      InputsAtCompileTime = NX,
      ValuesAtCompileTime = NY
    };
    
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    int m_inputs, m_values;

    BaseFunctor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    BaseFunctor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
  };
}

#endif    // REFINEMENT_BASE_FUNCTOR_HPP