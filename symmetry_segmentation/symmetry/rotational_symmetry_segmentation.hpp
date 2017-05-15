// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef ROTATIONAL_SYMMETRY_SEGMENTATION_HPP
#define ROTATIONAL_SYMMETRY_SEGMENTATION_HPP

// Symmetry
#include <symmetry/rotational_symmetry_segmentation.h>
#include <symmetry/rotational_symmetry_scoring.hpp>
#include <segmentation.hpp>

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetrySegmentation<PointT>::RotationalSymmetrySegmentation () :
  params_(),
  cloud_ (new pcl::PointCloud<PointT>),
  cloud_ds_ (new pcl::PointCloud<PointT>)
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetrySegmentation<PointT>::RotationalSymmetrySegmentation (const sym::RotSymSegParams &params) :
  params_ (params),
  cloud_ (new pcl::PointCloud<PointT>),
  cloud_ds_ (new pcl::PointCloud<PointT>)
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::RotationalSymmetrySegmentation<PointT>::~RotationalSymmetrySegmentation ()
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::RotationalSymmetrySegmentation<PointT>::setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{ 
  cloud_ = cloud; 
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::RotationalSymmetrySegmentation<PointT>::setInputOcuppancyMap  (const OccupancyMapConstPtr &occupancy_map)
{ 
  occupancy_map_ = occupancy_map;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::RotationalSymmetrySegmentation<PointT>::setInputSymmetries  (const std::vector<sym::RotationalSymmetry> &symmetries)
{ 
  symmetries_ = symmetries;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::RotationalSymmetrySegmentation<PointT>::setParameters  (const RotSymSegParams &params)
{
  params_ = params;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline bool
sym::RotationalSymmetrySegmentation<PointT>::segment ()
{
  //--------------------------------------------------------------------------
  // Entry checks
  
  if (cloud_->size() == 0)
  {
    std::cout << "[sym::RotationalSymmetrySegmentation::segment] input cloud is empty." << std::endl;
    return false;
  }
  
  if (!occupancy_map_)
  {
    std::cout << "[sym::ReflectionalSymmetryDetection::segment] occupancy map is not set." << std::endl;
    return false;
  }  
  
  if (symmetries_.size() == 0)
  {
    std::cout << "[sym::RotationalSymmetrySegmentation::segment] input symmetries are empty." << std::endl;
    return false;
  }
  
  cloud_ds_->clear();
  downsample_map_.clear();
  point_symmetry_scores_.clear();
  point_occlusion_scores_.clear();
  point_perpendicular_scores_.clear();
  adjacency_.clear();
  fg_weights_.clear();
  bg_weights_.clear();
  binary_weights_.clear();
  cut_edges_.clear();
  segments_.clear();
  segments_ds_.clear();
  symmetry_scores_.clear();
  occlusion_scores_.clear();
  smoothness_scores_.clear();
  segment_filtered_ids_.clear();
  
  //----------------------------------------------------------------------------
  // Downsample input pointcloud and create a search tree
  
  // Downsample the cloud
  if (params_.voxel_size <= 0.0f)
    *cloud_ds_ = *cloud_;
  else
  {
    utl::Downsample<PointT> dc;
    dc.setInputCloud(cloud_);
    dc.setDownsampleMethod(utl::Downsample<PointT>::AVERAGE); 
    dc.setLeafSize(params_.voxel_size);
    dc.filter(*cloud_ds_);
    dc.getDownsampleMap(downsample_map_);
  }

  //----------------------------------------------------------------------------
  // Compute binary weights
  
  if (!utl::cloudAdjacencyWeights<PointT> ( cloud_ds_,
                                            params_.aw_radius,
                                            params_.aw_num_neighbors,
                                            params_.aw_sigma_convex,
                                            params_.aw_sigma_concave,
                                            adjacency_))
    return false;
  
  binary_weights_ = adjacency_;
  for (size_t edgeId = 0; edgeId < adjacency_.getNumEdges(); edgeId++)
  {
    float weight;    
    if (!binary_weights_.getEdgeWeight(edgeId, weight))
    {
      std::cout << "[sym::RotationalSymmetrySegmentation::segment] Could not get edge weight." << std::endl;
      return false;
    }
    
    if (!binary_weights_.setEdgeWeight(edgeId, weight * params_.bin_weight_importance))
    {
      std::cout << "[sym::RotationalSymmetrySegmentation::segment] Could not set edge weight." << std::endl;
      return false;
    }
  }
  
  //----------------------------------------------------------------------------
  // Do the rest of the processing in a parfor loop
      
  point_symmetry_scores_.resize(symmetries_.size());
  point_occlusion_scores_.resize(symmetries_.size());
  point_perpendicular_scores_.resize(symmetries_.size());
  fg_weights_.resize(symmetries_.size());
  bg_weights_.resize(symmetries_.size());
  cut_edges_.resize(symmetries_.size());
  segments_ds_.resize(symmetries_.size());
  segments_.resize(symmetries_.size());
  symmetry_scores_.resize(symmetries_.size());
  occlusion_scores_.resize(symmetries_.size());
  smoothness_scores_.resize(symmetries_.size());
  
  std::vector<bool> success (symmetries_.size(), true);
  
  #pragma omp parallel for
  for (size_t symId = 0; symId < symmetries_.size(); symId++)
  {
    //--------------------------------------------------------------------------
    // Compute point scores
    
    sym::rotSymCloudSymmetryScore<PointT>       ( *cloud_ds_,
                                                  symmetries_[symId],
                                                  point_symmetry_scores_[symId],
                                                  params_.min_normal_fit_angle,
                                                  params_.max_normal_fit_angle );
    sym::rotSymCloudOcclusionScore<PointT>      ( *cloud_ds_,
                                                  occupancy_map_,
                                                  symmetries_[symId],
                                                  point_occlusion_scores_[symId],
                                                  params_.min_occlusion_distance,
                                                  params_.max_occlusion_distance );
    sym::rotSymCloudPerpendicularScores<PointT> ( *cloud_ds_,
                                                  symmetries_[symId],
                                                  point_perpendicular_scores_[symId],
                                                  params_.max_perpendicular_angle
                                                );
  
    //--------------------------------------------------------------------------
    // Assemble unary weights
    
    fg_weights_[symId].resize(cloud_ds_->size());
    bg_weights_[symId].resize(cloud_ds_->size());
    
    for (size_t pointId = 0; pointId < cloud_ds_->size(); pointId++)
    {
      // Foreground weights
      // NOTE: a foreground point must satisfy BOTH the symmetry constraint AND the occlusion contraint - hence we MULTIPLY the terms
      fg_weights_[symId][pointId] = (1.0f - point_symmetry_scores_[symId][pointId]) * (1.0f - point_occlusion_scores_[symId][pointId]) * (1.0f - point_perpendicular_scores_[symId][pointId]); //  * (1.0f - point_perpendicular_scores_[symId][pointId]) *
      fg_weights_[symId][pointId] *= params_.fg_weight_importance;
      
      // Background weights
      // NOTE: a background point must fail EITHER the symmetry constraint OR the occlusion contraint - hence we ADD the terms
      bg_weights_[symId][pointId] = point_symmetry_scores_[symId][pointId] * (1.0f - point_perpendicular_scores_[symId][pointId]) + point_occlusion_scores_[symId][pointId];
      bg_weights_[symId][pointId] *= params_.bg_weight_importance / 2.0f;
    }
    
    //--------------------------------------------------------------------------
    // Segment

    if (!utl::segmentCloudFG  ( fg_weights_[symId], bg_weights_[symId], binary_weights_, downsample_map_, segments_ds_[symId], segments_[symId]))
      success[symId] = false;

    utl::getCutEdges(binary_weights_, segments_ds_[symId], cut_edges_[symId]);
    
    //--------------------------------------------------------------------------
    // Compute segment scores
    
    if (segments_ds_[symId].size() == 0)
    {
      symmetry_scores_[symId]   = 0.0f;
      occlusion_scores_[symId]  = 0.0f;
      smoothness_scores_[symId] = 0.0f;
    }
    else
    {
      // Symmetry scores
      std::vector<int> boundaryPointIds, nonBoundaryPointIds;
      utl::getCloudBoundary<PointT>(cloud_ds_, segments_ds_[symId], params_.voxel_size * 2.0f, boundaryPointIds, nonBoundaryPointIds);
      for (size_t pointIdIt = 0; pointIdIt < nonBoundaryPointIds.size(); pointIdIt++)
      {
        int pointId = nonBoundaryPointIds[pointIdIt];
        symmetry_scores_[symId]   += point_symmetry_scores_[symId][pointId];
      }
      symmetry_scores_[symId]   /= static_cast<float>(nonBoundaryPointIds.size());
      
      // Occlusion scores
      for (size_t pointIdIt = 0; pointIdIt < segments_ds_[symId].size(); pointIdIt++)
      {
        int pointId = segments_ds_[symId][pointIdIt];
        occlusion_scores_[symId]  += point_occlusion_scores_[symId][pointId];
      }
      
      occlusion_scores_[symId]  /= static_cast<float>(segments_ds_[symId].size());

      // Smoothness score
      // If segmentation is empty or whole cloud - smoothness score is 0
      if (segments_ds_[symId].size() == cloud_ds_->size())
      {
        smoothness_scores_[symId] = 0.0f;
      }
      // Otherwise compute the average cost of edges that were cut
      else
      {
        // Compute average cut edge weight
        float cutScore = 0;
        for (size_t edgeId = 0; edgeId < cut_edges_[symId].getNumEdges(); edgeId++)
        {
          float edgeWeight;
          if (!cut_edges_[symId].getEdgeWeight(edgeId, edgeWeight))
          {
            std::cout << "[sym::RotationalSymmetrySegmentation::segment] Could not calculate smoothness scores." << std::endl;
            success[symId] = false;
          }
//           if (edgeWeight > 1.0f)
          cutScore += edgeWeight;
        }
        
        // Normalize
        smoothness_scores_[symId] = cutScore / static_cast<float>(segments_ds_[symId].size());
      }      
    }
  }
  
  for (size_t segId = 0; segId < success.size(); segId++)
    if (!success[segId])
      return false;

  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::filter ()
{
  segment_filtered_ids_.clear();
  
  for (size_t symId = 0; symId < segments_ds_.size(); symId++)
  {
    // Check if it's a good symmetry
    if (  symmetry_scores_[symId]     < params_.max_symmetry_score       &&
          occlusion_scores_[symId]    < params_.max_occlusion_score      &&
          smoothness_scores_[symId]   < params_.max_smoothness_score     &&
          segments_ds_[symId].size()  > params_.min_segment_size
        )
    {
      segment_filtered_ids_.push_back(symId);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getSegments (std::vector<std::vector<int> > &segments, std::vector<int> &segment_filtered_ids)
{
  segments = segments_;
  segment_filtered_ids = segment_filtered_ids_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getSegmentsDownsampled  ( typename pcl::PointCloud<PointT>::Ptr &cloud_ds,
                                                                            std::vector<std::vector<int> >        &segments_ds
                                                                          )
{
  cloud_ds.reset(new pcl::PointCloud<PointT>);
  *cloud_ds = *cloud_ds_;
  segments_ds = segments_ds_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getPointScores  ( std::vector<std::vector<float> >  &point_symmetry_scores,
                                                                    std::vector<std::vector<float> >  &point_occlusion_scores,
                                                                    std::vector<std::vector<float> >  &point_perpendicular_scores
                                                                  )
{
  point_symmetry_scores = point_symmetry_scores_;
  point_occlusion_scores = point_occlusion_scores_;
  point_perpendicular_scores = point_perpendicular_scores_;
}


////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getScores ( std::vector<float>  &symmetry_scores,
                                                              std::vector<float>  &occlusion_scores,
                                                              std::vector<float>  &smoothness_scores
                                                            )
{
  symmetry_scores   = symmetry_scores_;
  occlusion_scores  = occlusion_scores_;
  smoothness_scores = smoothness_scores_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getAdjacency  ( utl::GraphWeighted &adjacency)
{
  adjacency = adjacency_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::RotationalSymmetrySegmentation<PointT>::getGraphCutWeights  ( std::vector<std::vector<float> >        &fg_weights,
                                                                        std::vector<std::vector<float> >        &bg_weights,
                                                                        utl::GraphWeighted               &binary_weights,
                                                                        std::vector<utl::GraphWeighted>  &cut_edges
                                                                      )
{
  fg_weights = fg_weights_;
  bg_weights = bg_weights_;
  binary_weights = binary_weights_;
  cut_edges = cut_edges_;
}

#endif // ROTATIONAL_SYMMETRY_SEGMENTATION_HPP