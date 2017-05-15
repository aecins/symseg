// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef MATH_UTILITIES_HPP
#define MATH_UTILITIES_HPP

// STD includes
#include <iostream>
#include <cmath>
#include <random>

// Eigen includes
#include <eigen3/Eigen/Dense>

namespace utl
{
  /** \brief Calculate the remainder of division of two numbers. Unlike 
    * standard remainder functions provided by C++ this method return a true 
    * remainder that is bounded by [0, denom)
    *  \param[in] numer    numenator
    *  \param[in] denom    denominator
    *  \return remainder
    */
  template<class T>
  inline
  T remainder (const T numer, const T denom)
  {
    float result = std::fmod(numer, denom);
    if (result < 0)
      result = result + std::abs(denom);
    
    return result;
  }
  
  /** \brief Clamp a value from above and below.
    *  \param[in] value value to be clamped
    *  \param[in] minValue minimum value
    *  \param[in] maxValue maximum value
    *  \return bounded value
    *  \note behavior is undefined if maxValue < minValue
    */        
  template <typename Scalar>
  inline
  Scalar clampValue(const Scalar value, const Scalar minValue, const Scalar maxValue)
  {
    Scalar valueBounded = value;
    valueBounded = std::max(valueBounded, minValue);
    valueBounded = std::min(valueBounded, maxValue);
    return valueBounded;
  }

  /** \brief Clamp a value from above and below in circle.
    *  \param[in] value value to be clamped
    *  \param[in] minValue minimum value
    *  \param[in] maxValue maximum value
    *  \return bounded value
    *  \note behavior is undefined if maxValue < minValue
    */        
  template <typename Scalar>
  inline
  Scalar clampValueCircular(const Scalar value, const Scalar minValue, const Scalar maxValue)
  {
    return minValue + remainder(value - minValue,  maxValue - minValue + 1);
  }    
  
  /** \brief Count the number of values that are within the spcified bin range.
    * If a value is outside the provided range it is discarded.
    * Analog of MATLAB histc
    * (http://www.mathworks.com/help/matlab/ref/histc.html)
    *  \param[in] x input vector
    *  \param[in] bin_ranges number of bins in the histogram (monotonically non-decreasing)
    *  \param[out] bin_counts output histogram
    *  \note value a is assigned to bin b if a \in [b_min, b_max)
    */  
  inline
  void histc(const Eigen::VectorXd &x, const Eigen::VectorXd &bin_ranges, Eigen::VectorXd &bin_counts)
  {
    // Check input
    if (x.size() < 1 || bin_ranges.size() < 1)
    {
      std::cout << "[utl::histc] input vector and bin ranges size has to be greater than 0." << std::endl;
      abort();
    }
    
    // Count
    bin_counts = Eigen::VectorXd::Zero(bin_ranges.size());
    for (size_t i = 0; i < static_cast<size_t>(x.size()); i++)
    {
      // If current values is less than the smallest bin range skip it
      if (x[i] < bin_ranges[0])
        continue;
      
      // If current values is greater or equal to the largest bin range
      if (x[i] >= bin_ranges[bin_ranges.size()-1])
      {
        bin_counts[bin_ranges.size()-1]++;
        continue;
      }
      
      // Go over all other bin ranges
      for (size_t curBin = 0; curBin <static_cast<size_t>(bin_ranges.size()-1); curBin++)
      {
        if (x[i] < bin_ranges[curBin+1])
        {
          bin_counts[curBin]++;
          break;
        }
      }
    }
  }  
  
  /** \brief Compute the histgroam of the input vector
    *  \param[in] vector_in input vector
    *  \param[in] nbins number of bins in the histogram
    *  \param[out] hist_out output histogram
    *  \param[out] bin_centers center value of each bin
    */  
  inline
  void hist(const Eigen::VectorXd &x, int nbins, Eigen::VectorXd &hist_out, Eigen::VectorXd &bin_centers)
  {
    // Check that number of bins is at least 1
    if (nbins < 1)
    {
      std::cout << "[utl::hist] histogram requires at least one bin." << std::endl;
      abort();
    }
    
    // Create bins
    double minVal = x.minCoeff();
    double maxVal = x.maxCoeff();
    double step   = (maxVal - minVal) / static_cast<double>(nbins);
    Eigen::VectorXd bin_ranges  = Eigen::VectorXd::LinSpaced(nbins, minVal, maxVal-step);
    bin_centers                 = Eigen::VectorXd::LinSpaced(nbins, minVal+step/2, maxVal-step/2);    
    
    // Count
    utl::histc(x, bin_ranges, hist_out);
  }
      
  /** \brief Compute a normal pdf at a given set of data points
    * \param[in] x data points where rows correspond to dimensions and columns correspond to data points
    * \param[in] mean mean vector of the distribution
    * \param[in] cov covarinace matrix of the distribution
    * \return pdf values evaluated at data points
    */
  template <typename Scalar>
  inline
  Eigen::Matrix< Scalar, 1, Eigen::Dynamic > normpdf  ( const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x,
                                                        const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean,
                                                        const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov
                                                      )
  {
    // Get dimensionality of the data
    int dim = x.rows();
    int numPoints = x.cols();
    
    // Check input parameters
    if (mean.rows() != dim)
    {
      std::cout << "[smt::normpdf] mean vector must have same dimensionality as data point vector." << std::endl;
      abort();
    }
    
    if (cov.rows() != dim || cov.cols() != dim)
    {
      std::cout << "[smt::normpdf] covariance matrix must be square and have same dimensionality as data point vector." << std::endl;
      abort();
    }
  
    // Precompute constants
    Scalar denom = std::sqrt(pow(2*M_PI, dim) * cov.determinant());
    Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov_invserse = cov.inverse() / -2;
    Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x_demeaned = x - mean.replicate(1, numPoints);
    
    // Compute pdf values
    Eigen::Matrix< Scalar, 1, Eigen::Dynamic > pdf (numPoints);
    for (size_t pointId = 0; pointId < numPoints; pointId++)
      pdf.col(pointId) = (x_demeaned.col(pointId).transpose() * cov_invserse * x_demeaned.col(pointId));
    pdf = pdf.array().exp();
    pdf /= denom;
    
    return pdf;
  }
  
  /** \brief Compute a normal pdf at a given set of data points
    * \param[in] x data points where rows correspond to dimensions and columns correspond to data points
    * \param[in] mean mean value (same for all dimensions)
    * \param[in] cov covarinace value (same for all dimensions)
    * \return pdf values evaluated at data points
    */
  template <typename Scalar>
  inline
  Eigen::Matrix< Scalar, 1, Eigen::Dynamic > normpdf  ( const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x,
                                                        const Scalar mean_in,
                                                        const Scalar cov_in
                                                      )
  {
    int dim = x.rows();
    
    // Generate mean vector
    Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean (dim);
    mean.resize(dim, 1);
    mean.fill(mean_in);
    
    // Generate covariance matrix
    Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov (dim, dim);
    cov.fill(0.0f);
    for (size_t i = 0; i < dim; i++)
      cov(i,i) = cov_in;
    
    return normpdf<Scalar>(x, mean, cov);
  }

  /** \brief Compute a normal pdf at a given set of data points
    * \param[in] x data points where rows correspond to dimensions and columns correspond to data points
    * \param[in] mean mean vector of the distribution
    * \param[in] cov covarinace value (same for all dimensions)
    * \return pdf values evaluated at data points
    */
  template <typename Scalar>
  inline
  Eigen::Matrix< Scalar, 1, Eigen::Dynamic > normpdf  ( const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x,
                                                        const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean,
                                                        const Scalar cov_in
                                                      )
  {
    int dim = x.rows();
    
    // Generate covariance matrix
    Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov (dim, dim);
    cov.fill(0.0f);
    for (size_t i = 0; i < dim; i++)
      cov(i,i) = cov_in;
    
    return normpdf<Scalar>(x, mean, cov);
  }
  
  /** \brief Compute a normal pdf at a given set of data points
    * \param[in] x data points where rows correspond to dimensions and columns correspond to data points
    * \param[in] mean mean value (same for all dimensions)
    * \param[in] cov covarinace matrix of the distribution
    * \return pdf values evaluated at data points
    */
  template <typename Scalar>
  inline
  Eigen::Matrix< Scalar, 1, Eigen::Dynamic > normpdf  ( const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> x,
                                                        const Scalar mean_in,
                                                        const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov
                                                      )
  {
    int dim = x.rows();
  
    // Generate mean vector
    Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean (dim);
    mean.resize(dim, 1);
    mean.fill(mean_in);
  
    return normpdf<Scalar>(x, mean, cov);
  }
  
  /** \brief Generate samples from Multivariate Notmal Distribution
    *  \param num_samples number of samples to generate
    *  \param mu mean
    *  \param covar covariance
    *  \return matrix where each column is a sample
    */    
  inline
  Eigen::MatrixXd mvnrnd(const int num_samples, const Eigen::VectorXd &mean, const Eigen::MatrixXd &covar)
  {
    // Check that covariance matrix is square
    if (covar.cols() != covar.rows())
    {
      std::cout << "[utl::mvnrnd] covariance matrix must be square." << std::endl;
      abort();
    }
    
    // Check that covariance and mean have same dimensions
    if (covar.cols() != mean.size())
    {
      std::cout << "[utl::mvnrnd] covariance and mean must have same dimensions." << std::endl;
      abort();
    }
    
    // Check that covariance matrix is symmetric
    if (covar != covar.transpose())
    {
      std::cout << "[utl::mvnrnd] covariance matrix must by symmetric." << std::endl;
      abort();
    }
      
    // Decompose the covariance matrix
    int nDim = mean.size();        
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
    if (eigenSolver.info() != Eigen::Success)
    {
      std::cout << "[utl::mvnrnd] Something went wrong in eigen decomposition." << std::endl;
      abort();
    }
    // If there are negative eigenvalues then covariance matrix is not positive semidefinite
    else if (eigenSolver.eigenvalues().minCoeff() < 0)
    {
      std::cout << "[utl::mvnrnd] Covariance matrix is not positive semidefinite." << std::endl;
      abort();
    }
    
    Eigen::MatrixXd transform(nDim, nDim);
    transform = eigenSolver.eigenvectors() 
              * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    
    // Generate random samples ~N(0,1)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> distribution(0.0,1.0);
      
    Eigen::MatrixXd samples(nDim, num_samples);
    for (size_t col = 0; col < static_cast<size_t>(num_samples); col++)
      for (size_t i=0; i < static_cast<size_t>(nDim); i++)
        samples(i, col) = distribution(gen);
    
    // Transform samples
    samples = transform * samples + mean.replicate(1, num_samples);
      
    return samples;
  }
  
  /** \brief Compute rowwise mean of the sample
    *  \param samples matrix containing the samples. Each column is a sample
    *  \return mean of the sample
    */
  template <class Scalar>
  inline
  Eigen::Matrix< Scalar, Eigen::Dynamic, 1> mean(const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> &samples)
  {
    return samples.rowwise().mean();
  }

  /** \brief Compute the covariance of sample.
    *  \param samples matrix containing the samples. Each column is a sample
    *  \return covariance of the sample
    */
  template <class Scalar>
  inline
  Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> cov(const Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> &samples)
  {
    Eigen::Matrix< Scalar, Eigen::Dynamic, Eigen::Dynamic> centered = samples.colwise() - samples.rowwise().mean();
    return centered * centered.adjoint() / (samples.cols() - 1);    
  }
  
  /** \brief Compute mean of an std::vector.
    *  \param v    input vector
    *  \return mean of the vector
    */
  template <class Scalar>
  inline
  double mean(const std::vector<Scalar> &v)
  {
    Scalar sum = std::accumulate(v.begin(), v.end(), 0.0);
    return static_cast<double> (sum) / static_cast<double> (v.size());
  }

  /** \brief Compute median value of an std::vector.
    *  \param v    input vector
    *  \return median of the vector
    */
  template <class Scalar>
  inline
  Scalar median(const std::vector<Scalar> &v)
  {
    std::vector<Scalar> v_copy = v;
    std::nth_element(v_copy.begin(), v_copy.begin() + v_copy.size()/2, v_copy.end());
    return v_copy[v_copy.size()/2];
  }
}

#endif  // MATH_UTILITIES_HPP
