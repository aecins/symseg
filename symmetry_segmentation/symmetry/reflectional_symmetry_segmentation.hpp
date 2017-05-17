// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REFLECTIONAL_SYMMETRY_SEGMENTATION_HPP
#define REFLECTIONAL_SYMMETRY_SEGMENTATION_HPP

// Symmetry
#include <symmetry/reflectional_symmetry_segmentation.h>
#include <symmetry/reflectional_symmetry_scoring.hpp>
#include <segmentation.hpp>

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetrySegmentation<PointT>::ReflectionalSymmetrySegmentation () :
  params_(),
  cloud_ (new pcl::PointCloud<PointT>),
  cloud_ds_ (new pcl::PointCloud<PointT>)
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetrySegmentation<PointT>::ReflectionalSymmetrySegmentation (const sym::ReflSymSegParams &params) :
  params_ (params),
  cloud_ (new pcl::PointCloud<PointT>),
  cloud_ds_ (new pcl::PointCloud<PointT>)
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
sym::ReflectionalSymmetrySegmentation<PointT>::~ReflectionalSymmetrySegmentation ()
{}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud)
{ 
  cloud_ = cloud; 
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::setInputOcuppancyMap  (const OccupancyMapConstPtr &occupancy_map)
{ 
  occupancy_map_ = occupancy_map;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::setInputTablePlane (const Eigen::Vector4f &table_plane)
{ 
  table_plane_ = table_plane;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::setInputSymmetries  (const std::vector<sym::ReflectionalSymmetry> &symmetries, const std::vector<std::vector<int> > &symmetry_support)
{ 
  symmetries_ = symmetries;
  symmetry_support_segments_ = symmetry_support;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::setParameters  (const ReflSymSegParams &params)
{
  params_ = params;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline bool
sym::ReflectionalSymmetrySegmentation<PointT>::segment ()
{
  //--------------------------------------------------------------------------
  // Entry checks
  
  if (cloud_->size() == 0)
  {
    std::cout << "[sym::ReflectionalSymmetrySegmentation::segment] input cloud is empty." << std::endl;
    return false;
  }
  
  if (!occupancy_map_)
  {
    std::cout << "[sym::ReflectionalSymmetryDetection::segment] occupancy map is not set." << std::endl;
    return false;
  }  
  
  if (symmetries_.size() == 0)
  {
    std::cout << "[sym::ReflectionalSymmetrySegmentation::segment] input symmetries are empty." << std::endl;
    return false;
  }
  
  cloud_ds_->clear();
  downsample_map_.clear();
  point_symmetry_scores_.clear();
  point_occlusion_scores_.clear();
  point_perpendicular_scores_.clear();
  correspondences_.clear();
  symmetric_adjacency_.clear();
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
  symmetry_support_overlap_scores_.clear();
  segment_filtered_ids_.clear();
  segment_merged_ids_.clear();
  
  //----------------------------------------------------------------------------
  // Downsample input pointcloud and create a search tree
  
  // Downsample the cloud
  if (params_.voxel_size <= 0.0f)
  {
    *cloud_ds_ = *cloud_;
    downsample_map_.resize(cloud_->size());
    for (size_t pointId = 0; pointId < cloud_->size(); pointId++)
      downsample_map_[pointId].push_back(pointId);
  }
  else
  {
    utl::Downsample<PointT> dc;
    dc.setInputCloud(cloud_);
    dc.setDownsampleMethod(utl::Downsample<PointT>::AVERAGE); 
    dc.setLeafSize(params_.voxel_size);
    dc.filter(*cloud_ds_);
    dc.getDownsampleMap(downsample_map_);
  }
  
  cloud_search_tree_.setInputCloud(cloud_);
  
  //----------------------------------------------------------------------------
  // Compute cloud adjacency
  
  if (!utl::cloudAdjacencyWeights<PointT> ( cloud_ds_,
                                            params_.aw_radius,
                                            params_.aw_num_neighbors,
                                            params_.aw_sigma_convex,
                                            params_.aw_sigma_concave,
                                            adjacency_))
    return false;
  
  for (size_t edgeId = 0; edgeId < adjacency_.getNumEdges(); edgeId++)
  {
    float weight;    
    if (!adjacency_.getEdgeWeight(edgeId, weight))
      return false;
    
    if (!adjacency_.setEdgeWeight(edgeId, weight * params_.adjacency_importance))
      return false;
  }
    
  //----------------------------------------------------------------------------
  // Do the rest of the processing in a parfor loop
      
  point_symmetry_scores_.resize(symmetries_.size());
  point_occlusion_scores_.resize(symmetries_.size());
  point_perpendicular_scores_.resize(symmetries_.size());
  correspondences_.resize(symmetries_.size());
  symmetric_adjacency_.resize(symmetries_.size());
  fg_weights_.resize(symmetries_.size());
  bg_weights_.resize(symmetries_.size());
  binary_weights_.resize(symmetries_.size());
  cut_edges_.resize(symmetries_.size());
  segments_ds_.resize(symmetries_.size());
  segments_.resize(symmetries_.size());
  symmetry_scores_.resize(symmetries_.size());
  occlusion_scores_.resize(symmetries_.size());
  smoothness_scores_.resize(symmetries_.size());
  symmetry_support_overlap_scores_.resize(symmetries_.size());
  
  std::vector<bool> success (symmetries_.size(), true);
  
  std::vector<int> cloudBoundaryPointIds, cloudDSBoundaryPointIds, nonBoundaryPointIds;
  utl::getCloudBoundary<PointT>(cloud_, std::max(params_.voxel_size, 0.005f) * 2.0f, cloudBoundaryPointIds, nonBoundaryPointIds);
  utl::getCloudBoundary<PointT>(cloud_ds_, std::max(params_.voxel_size, 0.005f) * 2.0f, cloudDSBoundaryPointIds, nonBoundaryPointIds);

  #pragma omp parallel for
  for (size_t symId = 0; symId < symmetries_.size(); symId++)
  {
    //--------------------------------------------------------------------------
    // Get symmetry support mask
    
    std::vector<bool> symmetrySupportMask (cloud_ds_->size(), false);
    for (size_t pointIdIt = 0; pointIdIt < symmetry_support_segments_[symId].size(); pointIdIt++)
      symmetrySupportMask[symmetry_support_segments_[symId][pointIdIt]] = true;
    
    //--------------------------------------------------------------------------
    // Compute point scores
    
    sym::reflSymPointSymmetryScores<PointT> ( cloud_search_tree_,
                                              *cloud_ds_,
//                                               table_plane_,
                                              std::vector<int>(),
                                              std::vector<int>(),
//                                               cloudBoundaryPointIds,
//                                               cloudDSBoundaryPointIds, 
                                              symmetries_[symId],
                                              correspondences_[symId],
                                              point_symmetry_scores_[symId],
                                              params_.max_sym_corresp_reflected_distance,
                                              params_.min_normal_fit_angle,
                                              params_.max_normal_fit_angle
                                            );
    
    sym::reflSymPointOcclusionScores<PointT>  ( *cloud_ds_,
                                                occupancy_map_,
                                                symmetries_[symId],
                                                point_occlusion_scores_[symId],
                                                params_.min_occlusion_distance,
                                                params_.max_occlusion_distance
                                              );
    
    sym::reflSymPointPerpendicularScores<PointT>  ( *cloud_ds_,
                                                    symmetries_[symId],
                                                    point_perpendicular_scores_[symId]
                                                  );
    
    //--------------------------------------------------------------------------
    // Assemble unary weights
    
    fg_weights_[symId].resize(cloud_ds_->size(), 0.0f);
    bg_weights_[symId].resize(cloud_ds_->size());

    for (size_t pointId = 0; pointId < cloud_ds_->size(); pointId++)
    {
      bg_weights_[symId][pointId] = point_occlusion_scores_[symId][pointId];
    }
    
//     for (size_t crspId = 0; crspId < correspondences_[symId].size(); crspId++)
//     {
//       int pointId = correspondences_[symId][crspId].index_query;
//       if (point_symmetry_scores_[symId][crspId] >= 1.0f)
//         bg_weights_[symId][pointId] += 0.5f;
//     }
    
    // Foreground
    for (size_t crspId = 0; crspId < correspondences_[symId].size(); crspId++)
    {
      int pointId = correspondences_[symId][crspId].index_query;
      fg_weights_[symId][pointId] = (1.0f - point_symmetry_scores_[symId][crspId]);
      if (!symmetrySupportMask[pointId])
        fg_weights_[symId][pointId] *= (1.0f - point_perpendicular_scores_[symId][pointId] /** 0.9f*/);
      
      // NOTE: why are we not multiplying this by an occlusion score, just like we did for rotational symmetry?
    }
        
    // Weight the scores
    for (size_t pointId = 0; pointId < cloud_ds_->size(); pointId++)
    {
      fg_weights_[symId][pointId] *= params_.fg_weight_importance;
      bg_weights_[symId][pointId] *= params_.bg_weight_importance;
    }
    
    //--------------------------------------------------------------------------
    // Compute symmetric adjacency
    
    symmetric_adjacency_[symId].preallocateVertices(cloud_ds_->size());
    for (size_t crspId = 0; crspId < correspondences_[symId].size(); crspId++)
    {
      if (correspondences_[symId][crspId].distance < 0.0f)
        continue;
      
      // Only add symmetric adjacency if the correspondence score is high enough
      if (point_symmetry_scores_[symId][crspId] >= 1.0f)
        continue;
      
      int srcPointId = correspondences_[symId][crspId].index_query;
      int tgtPointId = correspondences_[symId][crspId].index_match;
      if (srcPointId != tgtPointId)
        symmetric_adjacency_[symId].addEdge(srcPointId, tgtPointId, (1.0f - point_symmetry_scores_[symId][crspId]) * params_.symmetric_adjacency_importance);
    }
    
    //--------------------------------------------------------------------------
    // Assemble binary weights
    
    binary_weights_[symId] = adjacency_;
    for (size_t edgeId = 0; edgeId < symmetric_adjacency_[symId].getNumEdges(); edgeId++)
    {
      utl::EdgeWeighted symAdjEdge;
      symmetric_adjacency_[symId].getEdge(edgeId, symAdjEdge);
      
      int adjEdgeId;
      if (!binary_weights_[symId].getEdgeId(symAdjEdge.vtx1Id_, symAdjEdge.vtx2Id_, adjEdgeId))
      {
        symAdjEdge.weight_ = params_.symmetric_adjacency_importance;
        binary_weights_[symId].addEdge(symAdjEdge);
      }
    }
    
    for (size_t edgeId = 0; edgeId < binary_weights_[symId].getNumEdges(); edgeId++)
    {
      float weight;
      if (!binary_weights_[symId].getEdgeWeight(edgeId, weight))
        success[symId];
      
      if (!binary_weights_[symId].setEdgeWeight(edgeId, weight * params_.bin_weight_importance))
        success[symId];
    }
    
    if (success[symId] == false)
      continue;

    //--------------------------------------------------------------------------
    // Segment

    if (!utl::segmentCloudFG  ( fg_weights_[symId], bg_weights_[symId], binary_weights_[symId], downsample_map_, segments_ds_[symId], segments_[symId]))
      success[symId] = false;

    if (success[symId] == false)
      continue;    
    
    utl::getCutEdges(binary_weights_[symId], segments_ds_[symId], cut_edges_[symId]);
    
    //--------------------------------------------------------------------------
    // Compute segment scores

    symmetry_scores_[symId]   = 0.0f;
    occlusion_scores_[symId]  = 0.0f;
    smoothness_scores_[symId] = 0.0f;
    symmetry_support_overlap_scores_[symId] = 0.0f;
    
    if (segments_ds_[symId].size() > 0)
    {
      // Symmetry scores
      
      // We must only use correspondences that belong to the segment!
      std::vector<bool> segment_ds_mask (cloud_ds_->size(), false);
      for (size_t pointIdIt = 0; pointIdIt < segments_ds_[symId].size(); pointIdIt++)
        segment_ds_mask[segments_ds_[symId][pointIdIt]] = true;
      
      int numInlierCorresp = 0;
      for (size_t crspId = 0; crspId < correspondences_[symId].size(); crspId++)
      {
        int srcPointId = correspondences_[symId][crspId].index_query;
        int tgtPointId = correspondences_[symId][crspId].index_match;
        if (segment_ds_mask[srcPointId] && segment_ds_mask[tgtPointId])
        {
          symmetry_scores_[symId] += point_symmetry_scores_[symId][crspId];
          numInlierCorresp++;
        }
      }
      
      if (numInlierCorresp > 0)
        symmetry_scores_[symId]   /= static_cast<float>(numInlierCorresp);
      else
        symmetry_scores_[symId] = 1.0f;
      
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
            std::cout << "[sym::ReflectionalSymmetrySegmentation::segment] Could not calculate smoothness scores." << std::endl;
            success[symId] = false;
          }
          cutScore += edgeWeight;
        }
        
        // Normalize
        smoothness_scores_[symId] = cutScore / static_cast<float>(segments_ds_[symId].size());
      }
      
      // Symmetry support overlap score
      symmetry_support_overlap_scores_[symId] = 0.0f;
      for (size_t pointIdIt = 0; pointIdIt < segments_ds_[symId].size(); pointIdIt++)
      {
        int pointId = segments_ds_[symId][pointIdIt];
        if (symmetrySupportMask[pointId])
          symmetry_support_overlap_scores_[symId] += 1.0f;
      }
      symmetry_support_overlap_scores_[symId] /= static_cast<float>(symmetry_support_segments_[symId].size());
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
void sym::ReflectionalSymmetrySegmentation<PointT>::filter ()
{
  segment_filtered_ids_.clear();
  
  for (size_t symId = 0; symId < segments_ds_.size(); symId++)
  {
    // Check if it's a good symmetry
    if (  symmetry_scores_[symId]     < params_.max_symmetry_score        &&
          occlusion_scores_[symId]    < params_.max_occlusion_score       &&
          smoothness_scores_[symId]   < params_.max_smoothness_score      &&
          segments_ds_[symId].size()  > params_.min_segment_size          &&
          symmetry_support_overlap_scores_[symId] > params_.min_symmetry_support_overlap
        )
    {
      segment_filtered_ids_.push_back(symId);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline void
sym::ReflectionalSymmetrySegmentation<PointT>::merge ()
{
  utl::Map segments_ds_filtered (segment_filtered_ids_.size());
  for (size_t segIdIt = 0; segIdIt < segment_filtered_ids_.size(); segIdIt++)
    segments_ds_filtered[segIdIt] = segments_ds_[segment_filtered_ids_[segIdIt]];
  
  utl::getSimilarSegments(segments_ds_filtered, segment_merged_ids_, params_.similar_segment_iou_);
  
  for (size_t segId = 0; segId < segment_merged_ids_.size(); segId++)
    for (size_t segInstId = 0; segInstId < segment_merged_ids_[segId].size(); segInstId++)
      segment_merged_ids_[segId][segInstId] = segment_filtered_ids_[segment_merged_ids_[segId][segInstId]];
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::ReflectionalSymmetrySegmentation<PointT>::getSegments ( std::vector<std::vector<int> > &segments,
                                                                  std::vector<int> &segment_filtered_ids,
                                                                  std::vector<std::vector<int> > &segment_merged_ids
                                                                )
{
  segments = segments_;
  segment_filtered_ids = segment_filtered_ids_;
  segment_merged_ids = segment_merged_ids_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::ReflectionalSymmetrySegmentation<PointT>::getSegmentsDownsampled  ( typename pcl::PointCloud<PointT>::Ptr &cloud_ds,
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
void sym::ReflectionalSymmetrySegmentation<PointT>::getPointScores  ( std::vector<std::vector<float> >  &point_symmetry_scores,
                                                                      std::vector<std::vector<float> >  &point_occlusion_scores,
                                                                      std::vector<std::vector<float> >  &point_perpendicular_scores,
                                                                      std::vector<pcl::Correspondences> &correspondences
                                                                    )
{
  point_symmetry_scores = point_symmetry_scores_;
  point_occlusion_scores = point_occlusion_scores_;
  point_perpendicular_scores = point_perpendicular_scores_;
  correspondences = correspondences_;
}


////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::ReflectionalSymmetrySegmentation<PointT>::getScores ( std::vector<float>  &symmetry_scores,
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
void sym::ReflectionalSymmetrySegmentation<PointT>::getAdjacency  ( utl::GraphWeighted &adjacency, std::vector<utl::GraphWeighted> &symmetric_adjacency)
{
  adjacency = adjacency_;
  symmetric_adjacency = symmetric_adjacency_;
}

////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
inline
void sym::ReflectionalSymmetrySegmentation<PointT>::getGraphCutWeights  ( std::vector<std::vector<float> >        &fg_weights,
                                                                          std::vector<std::vector<float> >        &bg_weights,
                                                                          std::vector<utl::GraphWeighted>  &binary_weights,
                                                                          std::vector<utl::GraphWeighted>  &cut_edges
                                                                        )
{
  fg_weights = fg_weights_;
  bg_weights = bg_weights_;
  binary_weights = binary_weights_;
  cut_edges = cut_edges_;
}

#endif // REFLECTIONAL_SYMMETRY_SEGMENTATION_HPP