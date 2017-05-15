// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REGION_GROWING_SMOOTHNESS_HPP
#define REGION_GROWING_SMOOTHNESS_HPP

#include "region_growing_smoothness.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
utl::RegionGrowingSmoothness<PointT>::RegionGrowingSmoothness () :
  normal_angle_threshold_ (0.0f),
  normals_consistently_oriented_ (false)
{
  updateBinaryConditionFunction();  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
utl::RegionGrowingSmoothness<PointT>::~RegionGrowingSmoothness ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowingSmoothness<PointT>::setNormalAngleThreshold (const float threshold)
{
  normal_angle_threshold_ = threshold;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
utl::RegionGrowingSmoothness<PointT>::getNormalAngleThreshold () const
{
  return normal_angle_threshold_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowingSmoothness<PointT>::setConsistentNormals (const bool normals_consistently_oriented)
{
  normals_consistently_oriented_ = normals_consistently_oriented;
  updateBinaryConditionFunction();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline bool
utl::RegionGrowingSmoothness<PointT>::getConsistentNormals () const
{
  return normals_consistently_oriented_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowingSmoothness<PointT>::updateBinaryConditionFunction ()
{
  if (normals_consistently_oriented_)
    setBinaryConditionFunction( boost::bind(&RegionGrowingSmoothness<PointT>::binaryConditionConsistent, this, _1, _2, _3) );
  else
    setBinaryConditionFunction( boost::bind(&RegionGrowingSmoothness<PointT>::binaryConditionNonConsistent, this, _1, _2, _3) );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowingSmoothness<PointT>::reorderIndicesCurvature ()
{
  if (!indices_ || indices_->empty())
  {
    indices_->resize(input_->size());
    for (size_t pointId = 0; pointId < indices_->size(); pointId++)
      (*indices_)[pointId] = pointId;
  }
  
  // Extract curvature values into a vector
  std::vector<float> curvature (indices_->size());
  for (size_t pointIdIt = 0; pointIdIt < indices_->size(); pointIdIt++)
    curvature[pointIdIt] = input_->points[(*indices_)[pointIdIt]].curvature;
    
  // Reorder indices in increasing order of curvature
  std::vector<float> curvature_sorted;
  std::vector<size_t> order;
  utl::sort(curvature, curvature_sorted, order, utl::ASCENDING);
  *indices_ = utl::reorder(*indices_, order);  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
utl::RegionGrowingSmoothness<PointT>::binaryConditionConsistent (const PointT& p1, const PointT& p2, float dist_squared) const
{
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  Eigen::Vector3f n2 = p2.getNormalVector3fMap();
    
  float angle = std::acos (utl::clampValue (n1.dot (n2), 0.0f, 1.0f));
  
  return angle < normal_angle_threshold_;
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// template <typename PointT> bool
// utl::RegionGrowingSmoothness<PointT>::binaryConditionConsistent (const PointT& p1, const PointT& p2, float dist_squared) const
// {
//   Eigen::Vector3f n1 = p1.getNormalVector3fMap();
//   Eigen::Vector3f n2 = p2.getNormalVector3fMap();
//     
//   float angle = std::acos (utl::clampValue (n1.dot (n2), 0.0f, 1.0f));
//   if (n1.dot(p1.getVector3fMap()-p2.getVector3fMap()) > 0)
//     angle = angle / 2.7;
//   
//   return angle < normal_angle_threshold_;
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
utl::RegionGrowingSmoothness<PointT>::binaryConditionNonConsistent (const PointT& p1, const PointT& p2, float dist_squared) const
{
  Eigen::Vector3f n1 = p1.getNormalVector3fMap();
  Eigen::Vector3f n2 = p2.getNormalVector3fMap();
    
  float angle = std::acos (utl::clampValue (std::abs (n1.dot (n2)), 0.0f, 1.0f));
  return angle < normal_angle_threshold_;  
}
 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline bool
utl::RegionGrowingSmoothness<PointT>::prepareForSegmentation ()
{  
  // If user forgot to pass point cloud or if it is empty
  if ( normal_angle_threshold_ < 0.0f )
  {
    std::cout << "[utl::RegionGrowingSmoothness::prepareForSegmentation] normal variation threhsold must be non-negative!" << std::endl;
    return (false);
  }
  
  bool good = utl::RegionGrowing<PointT>::prepareForSegmentation();
  
  if (good)
  {
    this->reorderIndicesCurvature();
    return true;
  }
  else
  {
    return false;
  }
}

#endif  // REGION_GROWING_SMOOTHNESS_HPP