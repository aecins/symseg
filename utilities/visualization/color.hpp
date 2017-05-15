// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef PCL_VISUALIZATION_COLOR_HPP
#define PCL_VISUALIZATION_COLOR_HPP

// STD
#include <iostream>

// PCL
#include <pcl/io/point_cloud_image_extractors.h>

// IF PCL my version
//#include <pcl/io/impl/point_cloud_image_extractors.hpp>

// IF PCL 1.8 trunk
#include <pcl/common/colors.h>

// CPP tools
#include <visualization/color.h>

namespace utl
{    
  //--------------------------------------------------------------------------
  // Color generators
  //--------------------------------------------------------------------------

  /** \brief Generate a random color
    *  \return color      RGB color triplet corresponding to the element. Colors are in [0,1] range
    */
  inline
  Color getRandomColor()
  {
    Color color;
    std::cout << "[utl::pclvis::getRandomColor] not implemented yet!" << std::endl;
    std::abort();
    
    return color;
  }
  
  /** \brief Generate a random color
    *  \param[in]  id      index of the element
    *  \return color      RGB color triplet corresponding to the element. Colors are in [0,1] range
    */
  inline
  Color getGlasbeyColor(const int id)
  {
    Color color;
    
//       if (maxId > static_cast<int>(pcl::GLASBEY_LUT_SIZE))
//       {
//         std::cout << "[smt::pclvis::getGlasbeyColor] Maximum number of Glasbey colors is " << pcl::GLASBEY_LUT_SIZE << ", you requested " << maxId << std::endl;
//         std::cout << "[smt::pclvis::getGlasbeyColor] Some ids might have same colors." << std::endl;
//       }

//       // IF PCL my version
//       int tmpId = id % pcl::GLASBEY_LUT_SIZE;
//       color.r = static_cast<float>(pcl::GLASBEY_LUT[tmpId * 3 + 0]) / 255;
//       color.g = static_cast<float>(pcl::GLASBEY_LUT[tmpId * 3 + 1]) / 255;
//       color.b = static_cast<float>(pcl::GLASBEY_LUT[tmpId * 3 + 2]) / 255;
    
    // IF PCL 1.8 trunk      
    int tmpId = id % pcl::GlasbeyLUT().size();
    pcl::RGB color_pcl = pcl::GlasbeyLUT().at(tmpId);
    color.r = static_cast<float>(color_pcl.r) / 255;
    color.g = static_cast<float>(color_pcl.g) / 255;
    color.b = static_cast<float>(color_pcl.b) / 255;
    
    return color;
  }

  /** \brief Convert a single point with RGB information to grayscale
    *  \param[in,out]  point point to be converted to grayscale
    *  \note conversion is done using the formula used in OpenCV (http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html)
    */
  template <typename PointT>
  inline
  void rgb2gray(PointT &point)
  {
    uint8_t gray = static_cast<uint8_t> ( static_cast<float>(point.r) * 0.299 +
                                          static_cast<float>(point.r) * 0.587 +
                                          static_cast<float>(point.r) * 0.114 );
    point.r = gray;
    point.g = gray;
    point.b = gray;
  }
      
  /** \brief Convert color in pointcloud to grayscale
    *  \param[in,out]  cloud cloud to be converted to grayscale
    *  \note conversion is done using the formula used in OpenCV (http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html)
    */
  template <typename PointT>
  inline
  void rgb2gray(pcl::PointCloud<PointT> &cloud)
  {
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
      rgb2gray<PointT>(cloud.points[pointId]);
  }
  
  /** \brief Change the tint of a point to a given color
    *  \param[in,out]  point point to be converted to grayscale
    *  \param[in]      color color of the tint
    *  \param[in]      alpha amount of tint. 0 leaves point color unchanged while 1 replace it by input color. Default 0.5
    */
  template <typename PointT>
  inline
  void tintPoint(PointT &point, const Color &color, const float alpha = 0.3)
  {      
    point.r = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(point.r * (1-alpha)  +  color.r * alpha * 255.0f)));
    point.g = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(point.g * (1-alpha)  +  color.g * alpha * 255.0f)));
    point.b = static_cast<uint8_t>(std::min(255.0f, static_cast<float>(point.b * (1-alpha)  +  color.b * alpha * 255.0f)));
  }
  
  /** \brief Change the tint of a pointcloud to a given color
    *  \param[in,out]  cloud pointcloud to be converted to grayscale
    *  \param[in]      color color of the tint
    *  \param[in]      alpha amount of tint. 0 leaves point color unchanged while 1 replace it by input color. Default 0.5
    */
  template <typename PointT>
  inline
  void tintPointCloud(pcl::PointCloud<PointT> &cloud, const std::vector<int> &indices, const Color &color, const float alpha = 0.3)
  {
    for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
      tintPoint<PointT>(cloud.points[indices[pointIdIt]], color, alpha);
  }
  
  /** \brief Change the tint of a pointcloud to a given color
    *  \param[in,out]  cloud pointcloud to be converted to grayscale
    *  \param[in]      color color of the tint
    *  \param[in]      alpha amount of tint. 0 leaves point color unchanged while 1 replace it by input color. Default 0.5
    */
  template <typename PointT>
  inline
  void tintPointCloud(pcl::PointCloud<PointT> &cloud, const Color &color, const float alpha = 0.3)
  {
    std::vector<int> indices (cloud.size());
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
      indices[pointId] = pointId;
    
    tintPointCloud<PointT>(cloud, indices, color, alpha);
  }
}


#endif  // PCL_VISUALIZATION_COLOR_HPP