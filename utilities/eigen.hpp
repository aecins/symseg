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

#ifndef EIGEN_UTILITIES_HPP
#define EIGEN_UTILITIES_HPP

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

namespace utl
{
  /** \brief Write matrix to a file in binary mode.
    *  \param filename output file name
    *  \param matrix matrix
    *  \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
    */
  template<class Matrix>
  inline
  bool writeBinary(const std::string filename, const Matrix& matrix)
  {
    std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if (out.is_open())
    {
      typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
      out.write((char*) (&rows), sizeof(typename Matrix::Index));
      out.write((char*) (&cols), sizeof(typename Matrix::Index));
      out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
      out.close();
      return true;
    }
    else
    {
      std::cout << "[utl::eigen::writeBinary] Colud not open file '" << filename << "' for writing\n";
      return false;
    }
  }
  
  /** \brief Read a matrix from a binary file.
    *  \param filename input file name
    *  \param matrix matrix
    *  \note http://stackoverflow.com/questions/25389480/how-to-write-read-an-eigen-matrix-from-binary-file
    */  
  template<class Matrix>
  inline
  bool readBinary(const std::string filename, Matrix& matrix)
  {
    std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
    if (!in.is_open())
    {
      std::cout << "[utl::eigen::readBinary] Colud not open file '" << filename << "' for reading." << std::endl;
      return false;
    }
    
    typename Matrix::Index rows=0, cols=0;
    in.read((char*) (&rows),sizeof(typename Matrix::Index));
    in.read((char*) (&cols),sizeof(typename Matrix::Index));
    matrix.resize(rows, cols);
    in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
    in.close();
    return true;
  }
  
  /** \brief Write matrix to a file in ASCII mode.
    *  \param filename output file name
    *  \param matrix matrix
    *  \return TRUE if file written successfully
    */
  template<class Matrix>
  inline
  bool writeASCII(const std::string filename, const Matrix& matrix)
  {
    std::ofstream out(filename.c_str(), std::ios::out);
    if (out.is_open())
    {
      out << matrix << "\n";
      out.close();
      return true;
    }
    else
    {
      std::cout << "[utl::eigen::writeASCII] Colud not open file '" << filename << "' for writing\n";
      return false;
    }
  }
  
  /** \brief Read a matrix from an ASCII file.
    *  \param filename input file name
    *  \param matrix matrix
    *  \return TRUE if file read successfully
    *  \note Adapted from http://perso.ensta-paristech.fr/~stulp/dmpbbo/EigenFileIO_8tpp_source.html
    */  
  template<class Matrix>
  inline
  bool readASCII(const std::string filename, Matrix& matrix)
  {
    std::ifstream in(filename.c_str(), std::ios::in);
    if (!in.is_open())
    {
      std::cout << "[utl::eigen::readASCII] Colud not open file '" << filename << "' for reading." << std::endl;
      return false;
    }
    
    // Read file contents into a vector
    std::string line;
    typename Matrix::Scalar d;
    
    std::vector<typename Matrix::Scalar> v;
    int n_rows = 0;
    while (getline(in, line))
    {
      ++n_rows;
      std::stringstream input_line(line);
      while (!input_line.eof())
      {
        input_line >> d;
        v.push_back(d);
      }
    }
    in.close();
    
    // Construct matrix
    int n_cols = v.size()/n_rows;
    matrix = Eigen::Matrix<typename Matrix::Scalar,Eigen::Dynamic,Eigen::Dynamic>(n_rows,n_cols);
    
    for (int i=0; i<n_rows; i++)
      for (int j=0; j<n_cols; j++)
        matrix(i,j) = v[i*n_cols + j];
      
    return true;
  }
}

#endif  // EIGEN_UTILITIES_HPP