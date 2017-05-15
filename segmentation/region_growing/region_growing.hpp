// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef REGION_GROWING_HPP
#define REGION_GROWING_HPP

#include "region_growing.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
utl::RegionGrowing<PointT>::RegionGrowing () :
    searcher_ (),
    search_radius_ (0.0),
    num_neighbors_ (0),
    unary_condition_function_ (),    
    binary_condition_function_ (),
    min_valid_unary_neighbor_fraction_ (0.0),
    min_valid_binary_neighbor_fraction_ (0.0),
    min_cluster_size_ (1),
    max_cluster_size_ (std::numeric_limits<int>::max ())
{
}
      
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
utl::RegionGrowing<PointT>::~RegionGrowing ()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  pcl::PCLBase<PointT>::setInputCloud (cloud);
  if (fake_indices_)
    indices_->clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setUnaryConditionFunction (bool (*unary_condition_function) (const PointT&)) 
{
  unary_condition_function_ = unary_condition_function;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setUnaryConditionFunction (unaryFunction unary_condition_function) 
{
  unary_condition_function_ = unary_condition_function;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setBinaryConditionFunction (bool (*binary_condition_function) (const PointT&, const PointT&, float)) 
{
  binary_condition_function_ = binary_condition_function;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setBinaryConditionFunction (binaryFunction binary_condition_function) 
{
  binary_condition_function_ = binary_condition_function;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setSearchRadius (float search_radius)
{
  search_radius_ = search_radius;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
utl::RegionGrowing<PointT>::getSearchRadius () const
{
  return search_radius_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setNumberOfNeighbors (int num_neighbors)
{
  num_neighbors_ = num_neighbors;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline int
utl::RegionGrowing<PointT>::getNumberOfNeighbors () const
{
  return num_neighbors_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setMinValidUnaryNeighborsFraction (float valid_unary_neighbor_fraction)
{
  if (valid_unary_neighbor_fraction > 1.0f || valid_unary_neighbor_fraction < 0.0f)
  {
    std::cout << "[utl::RegionGrowing::setMinValidUnaryNeighborsFraction] minimum fraction of points satisfying the unary condition has to be in [0, 1] interval." << std::endl;
    abort();
  }
  
  min_valid_unary_neighbor_fraction_ = valid_unary_neighbor_fraction;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
utl::RegionGrowing<PointT>::getMinValidUnaryNeighborsFraction () const
{
  return min_valid_unary_neighbor_fraction_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setMinValidBinaryNeighborsFraction (float valid_binary_neighbor_fraction)
{
  if (valid_binary_neighbor_fraction > 1.0f || valid_binary_neighbor_fraction < 0.0f)
  {
    std::cout << "[utl::RegionGrowing::setMinValidBinaryNeighborsFraction] minimum fraction of points satisfying the binary condition has to be in [0, 1] interval." << std::endl;
    abort();
  }
  
  min_valid_binary_neighbor_fraction_ = valid_binary_neighbor_fraction;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline float
utl::RegionGrowing<PointT>::getMinValidBinaryNeighborsFraction () const
{
  return min_valid_unary_neighbor_fraction_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setMinSegmentSize (int min_segment_size)
{
  // Check input
  if (min_segment_size < 1)
  {
    std::cout << "[utl::RegionGrowing::setMinSegmentSize] segments must contain at least one point." << std::endl;
    abort();
  }    
  
  min_cluster_size_ = min_segment_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline int
utl::RegionGrowing<PointT>::getMinSegmentSize () const
{
  return min_cluster_size_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline void
utl::RegionGrowing<PointT>::setMaxSegmentSize (int max_segment_size)
{
  // Check input
  if (max_segment_size < 1)
  {
    std::cout << "[utl::RegionGrowing::setMaxSegmentSize] segments must contain at least one point." << std::endl;
    abort();
  }    
  
  max_cluster_size_ = max_segment_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline int
utl::RegionGrowing<PointT>::getMaxSegmentSize () const
{
  return max_cluster_size_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> inline void
utl::RegionGrowing<PointT>::segment (std::vector<std::vector<int> > &segments)
{
  // First check that segmentation is possible
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    std::cout << "[utl::RegionGrowing::segment] could not initialize segmentation." << std::endl;
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    std::cout << "[utl::RegionGrowing::segment] could not initialize segmentation." << std::endl;
    deinitCompute ();
    return;
  }
  
  // Prepare output (going to use push_back)
  segments.clear ();

  // Initialize the search class
  if (!searcher_)
  {
    if (input_->isOrganized ())
      searcher_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      searcher_.reset (new pcl::search::KdTree<PointT> ());
  }
  searcher_->setInputCloud (input_, indices_);  
  
  // Temp variables used by search class
  std::vector<int> nn_indices (num_neighbors_+1);
  std::vector<float> nn_distances (num_neighbors_+1);
  
  // Create a bool vector of processed point indices, and initialize it to false
  // Need to have it contain all possible points because radius search can not return indices into indices
  std::vector<bool> processed (input_->points.size (), false);

  // Process all points indexed by indices_
  for (int iii = 0; iii < static_cast<int> (indices_->size ()); ++iii)  // iii = input indices iterator
  {
    // If this point has already been processed or it doesn't satisfy unary constraint - ignore it
    if ((*indices_)[iii] == -1 || processed[(*indices_)[iii]])
      continue;
    
    if (unary_condition_function_ && !unary_condition_function_(input_->points[(*indices_)[iii]]))
      continue;

    // Set up a new growing cluster
    std::vector<int> current_cluster;
    int cii = 0;  // cii = cluster indices iterator

    // Add the point to the cluster
    current_cluster.push_back ((*indices_)[iii]);
    processed[(*indices_)[iii]] = true;

    // Process the current cluster (it can be growing in size as it is being processed)
    while (cii < static_cast<int> (current_cluster.size ()))
    {
      // Search for neighbors around the current seed point of the current cluster
      int num_neighbors_found;
      if (search_radius_ > 0 && num_neighbors_ > 0)
        num_neighbors_found = searcher_->radiusSearch (input_->points[current_cluster[cii]], search_radius_, nn_indices, nn_distances, num_neighbors_+1);
      else if (search_radius_ > 0 && num_neighbors_ == 0)
        num_neighbors_found = searcher_->radiusSearch (input_->points[current_cluster[cii]], search_radius_, nn_indices, nn_distances);
      else if (search_radius_ == 0 && num_neighbors_ > 0)
        num_neighbors_found = searcher_->nearestKSearch (input_->points[current_cluster[cii]], num_neighbors_+1, nn_indices, nn_distances);
      else
      {
        std::cout << "[utl::RegionGrowing::segment] unknown combination of search radius and number of neighbors." << std::endl;
        abort();
      }
      
      if (num_neighbors_found < 1)
      {
        cii++;
        continue;
      }

      // Process the neighbors
      std::vector<int> valid_point_indices;
      int numUnprocessed = 0;
      int numValidUnary = 0;
      int numValidBinary = 0;      
      for (int nii = 1; nii < static_cast<int> (nn_indices.size ()); ++nii)  // nii = neighbor indices iterator
      {
        // If this point has already been processed - ignore it
        if (nn_indices[nii] == -1 || processed[nn_indices[nii]])
          continue;
        numUnprocessed++;
        
        //  If this point does not satisfy unary constraint - ignore it
        if (unary_condition_function_ && !unary_condition_function_(input_->points[nn_indices[nii]]))
          continue;
        numValidUnary++;
        
        // If binary condition exists and is not satisfied - ignore this neighbor
        if (  binary_condition_function_ && !binary_condition_function_ (input_->points[current_cluster[cii]], input_->points[nn_indices[nii]], nn_distances[nii]))
          continue;
        numValidBinary++;
        
        valid_point_indices.push_back(nn_indices[nii]);
      }
      
      float validUnaryFraction = static_cast<float>(numValidUnary) / static_cast<float>(numUnprocessed);
      float validBinaryFraction = static_cast<float>(numValidBinary) / static_cast<float>(numUnprocessed);
      if (validUnaryFraction >= min_valid_unary_neighbor_fraction_ && validBinaryFraction >= min_valid_binary_neighbor_fraction_)
      {
        for (size_t nii = 0; nii < valid_point_indices.size(); nii++)
        {
          // Add the point to the cluster
          current_cluster.push_back (valid_point_indices[nii]);
          processed[valid_point_indices[nii]] = true;
        }
      }
      cii++;
    }

    // Save current cluster
    if (current_cluster.size() >= min_cluster_size_ && current_cluster.size() <= max_cluster_size_)
      segments.push_back(current_cluster);
  }

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> inline bool
utl::RegionGrowing<PointT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] input cloud is empty!" << std::endl;
    return (false);
  }

  if (indices_ && indices_->empty ())
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] input indices are empty!" << std::endl;
    return false;
  }  
  
  if (!unary_condition_function_ && !binary_condition_function_)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] Both unary and binary condition functions are empty! One of them has to be defined." << std::endl;
    return false;
  }

  if (min_valid_unary_neighbor_fraction_ > 1.0 || min_valid_unary_neighbor_fraction_ < 0.0)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] minimum fraction of neighbors satisfying unary condition has to be in [0, 1] interval!" << std::endl;
    return false;
  }

  if (min_valid_binary_neighbor_fraction_ > 1.0 || min_valid_binary_neighbor_fraction_ < 0.0)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] minimum fraction of neighbors satisfying binary condition has to be in [0, 1] interval!" << std::endl;
    return false;
  }  
  
  if (num_neighbors_ < 0)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] number of neighbors must be greater or equal than zero!" << std::endl;
    return false;
  }

  if (search_radius_ < 0.0f)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] search radius be greater or equal than zero!" << std::endl;
    return false;
  }
  
  // from here we check those parameters that are always valuable
  if (num_neighbors_ == 0 && search_radius_ == 0)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] both search radius and number of neighbors are equal to zero" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] Set them according to the following guidelines:" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]  - num_neighbors > 0, search_radius <= 0 :" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]    search for num_neighbors nearest neighbors" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]  - num_neighbors <= 0, search_radius > 0 :" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]    search for all neighbors within search_radius" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]  - num_neighbors > 0, search_radius > 0 :" << std::endl;
    std::cout << "[utl::RegionGrowing::prepareForSegmentation]    search for num_neighbors neareset neighbors within search_radius :" << std::endl;
    
    return (false);
  }
    
  if (min_cluster_size_ < 1)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] minimum segment size is less than one!" << std::endl;
    return (false);
  }
  
  if (max_cluster_size_ < 1)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] maximum segment size is less than one!" << std::endl;
    return (false);
  }

  if (max_cluster_size_ < min_cluster_size_)
  {
    std::cout << "[utl::RegionGrowing::prepareForSegmentation] maximum segment size is smaller than minimum segment size!" << std::endl;
    return (false);
  }
  
  return (true);
}

#endif  // REGION_GROWING_HPP