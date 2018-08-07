/*****************************************************************************/
/*  Copyright (c) 2017, Aleksandrs Ecins                                     */
/*  All rights reserved.                                                     */
/*                                                                           */
/*  Redistribution and use in source and binary forms, with or without       */
/*  modification, are permitted provided that the following conditions       */
/*  are met:                                                                 */
/*                                                                           */
/*  1. Redistributions of source code must retain the above copyright        */
/*  notice, this list of conditions and the following disclaimer.            */
/*                                                                           */
/*  2. Redistributions in binary form must reproduce the above copyright     */
/*  notice, this list of conditions and the following disclaimer in the      */
/*  documentation and/or other materials provided with the distribution.     */
/*                                                                           */
/*  3. Neither the name of the copyright holder nor the names of its         */
/*  contributors may be used to endorse or promote products derived from     */
/*  this software without specific prior written permission.                 */
/*                                                                           */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS      */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT        */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     */
/*  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*****************************************************************************/

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