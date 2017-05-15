// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

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