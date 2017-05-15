// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include <omp.h>
#include <mutex>

// Utilities
#include <pointcloud/pointcloud.hpp>
#include <graph/graph_algorithms.hpp>
#include <graph/min_cut.hpp>
#include <graph/bron_kerbosch.hpp>

namespace utl
{
  //----------------------------------------------------------------------------
  // Scene oversegmentation parameters
  //----------------------------------------------------------------------------
  
  struct SmoothSegParams
  {
    float voxel_size      = 0.005f;       // Scene will be downsampled to this resolution
    int min_segment_size  = 120;          // Minimum size of segment used as symmetry support (in metres)
    float max_iou         = 0.8f;         // Maximum intersection over union of two different segments
    
    // Smoothness parameters
    std::vector<std::pair<float, float> > smoothness  = { std::pair<float, float>(pcl::deg2rad(10.0f), 0.5f),
                                                          std::pair<float, float>(pcl::deg2rad(15.0f), 0.5f) };
  };  
  
  //----------------------------------------------------------------------------
  // Segment merging
  //----------------------------------------------------------------------------

  /** \brief Merge duplicate segments from multiple oversegmentations of a scene.
  * Two segments are considered duplicate if their intersection over overlap is
  * greater than a threshold. Individual input segmentations are expected to be
  * non-overlapping.
  *  \param[in]  segmentations   vector of segmentations of a scene. Each individual segmentation is assumed to contain non-overlapping segments.
  *  \param[out] segments_merged merged segments
  *  \param[in]  iou_threshold   minimum intersection over union score of two duplicate segments
  *  \return maximum value
  * NOTE: this function should be replaced by findSimilarSegments function.
  * after that, the calling applications can decide which criteria to use for 
  * segment merging.
  */        
  void  mergeDuplicateSegments  ( const std::vector<utl::Map> &segmentations, std::vector<std::vector<int> > &segmentation_merged_ids, const float iou_threshold = 0.9f)
  {
    std::mutex graphMutex;

    // Get number of segments
    int numSegs = 0;
    for (size_t segSetId = 0; segSetId < segmentations.size(); segSetId++)
      numSegs += segmentations[segSetId].size();
    
    // Construct a graph where vertices represent object segments and edges
    // indicate segments that are similar
    utl::Graph segmentSimilarityGraph (numSegs);
    
    for (size_t srcSegSetId = 0; srcSegSetId < segmentations.size()-1; srcSegSetId++)
    {
      utl::Map srcSegmentation = segmentations[srcSegSetId];
      for (size_t tgtSegSetId = srcSegSetId+1; tgtSegSetId < segmentations.size(); tgtSegSetId++)
      {
        utl::Map tgtSegmentation = segmentations[tgtSegSetId];
        
        #pragma omp parallel for
        for (size_t srcSegId = 0; srcSegId < srcSegmentation.size(); srcSegId++)
        {
          for (size_t tgtSegId = 0; tgtSegId < tgtSegmentation.size(); tgtSegId++)
          {
            int segIntersection = utl::vectorIntersection<int>(srcSegmentation[srcSegId], tgtSegmentation[tgtSegId]).size();
            int segUnion        = utl::vectorUnion<int>(srcSegmentation[srcSegId], tgtSegmentation[tgtSegId]).size();
            float iou           = static_cast<float>(segIntersection) / static_cast<float>(segUnion);
            
            if (iou > iou_threshold)
            {
              int srcSegLinId, tgtSegLinId;
              utl::vector2dSubToLinearId(segmentations, srcSegSetId, srcSegId, srcSegLinId);
              utl::vector2dSubToLinearId(segmentations, tgtSegSetId, tgtSegId, tgtSegLinId);
              graphMutex.lock();
              segmentSimilarityGraph.addEdge(srcSegLinId, tgtSegLinId);
              graphMutex.unlock();
            }
          }
        }
      }
    }

    // Find all connected components in the segment graph (this should be replaced by finding maximal cliques)
    utl::Map segmentCCs;
    segmentCCs = utl::getConnectedComponents  (segmentSimilarityGraph);
      
    // Select best hypothesis for each cluster
    segmentation_merged_ids.resize(segmentations.size());
    
    for (size_t clusterId = 0; clusterId < segmentCCs.size(); clusterId++)
    {
      int maxSize       = -1;
      int bestSegSetId  = -1;
      int bestSegId       = -1;
      for (size_t segLinIdIt = 0; segLinIdIt < segmentCCs[clusterId].size(); segLinIdIt++)
      {      
        int segLinId = segmentCCs[clusterId][segLinIdIt];
        int segSetId, segId;
        utl::vector2dLinearId2Sub(segmentations, segLinId, segSetId, segId);
        int segSize = segmentations[segSetId][segId].size();
        
        if (segSize > maxSize)
        {
          maxSize       = segSize;
          bestSegSetId  = segSetId;
          bestSegId     = segId;
        }
        
        if (bestSegSetId == -1 || bestSegId == -1)
          std::cout << "[utl::mergeDuplicateSegments] Could not select best hypothesis. Probably going to crash now." << std::endl;
      }
      segmentation_merged_ids[bestSegSetId].push_back(bestSegId);
    }
  }
  
  /** \brief Merge duplicate segments from multiple oversegmentations of a scene.
   * Two segments are considered duplicate if their intersection over overlap is
   * greater than a threshold. Individual input segmentations are expected to be
   * non-overlapping.
   *  \param[in]  segmentations   vector of segmentations of a scene. Each individual segmentation is assumed to contain non-overlapping segments.
   *  \param[out] segments_merged merged segments
   *  \param[in]  iou_threshold   minimum intersection over union score of two duplicate segments
   *  \return maximum value
   * NOTE: this function should be replaced by findSimilarSegments function.
   * after that, the calling applications can decide which criteria to use for 
   * segment merging.
   */        
  void  getSimilarSegments  ( const utl::Map &segments, std::vector<std::vector<int> > &segment_similarity, const float iou_threshold = 0.9f)
  {
    segment_similarity.clear();
    
    if (segments.size() == 0)
      return;
    
    // Construct a graph where vertices represent object segments and edges
    // indicate segments that are similar
    utl::Graph segmentSimilarityGraph (segments.size());

    for (size_t srcSegId = 0; srcSegId < segments.size()-1; srcSegId++)
    {
      for (size_t tgtSegId = srcSegId+1; tgtSegId < segments.size(); tgtSegId++)
      {
        int segIntersection = utl::vectorIntersection<int>(segments[srcSegId], segments[tgtSegId]).size();
        int segUnion        = utl::vectorUnion<int>(segments[srcSegId], segments[tgtSegId]).size();
        float iou           = static_cast<float>(segIntersection) / static_cast<float>(segUnion);
        
        if (iou > iou_threshold)
          segmentSimilarityGraph.addEdge(srcSegId, tgtSegId);
      }
    }

    // Find all connected components in the segment graph (this should be replaced by finding maximal cliques)
    std::list<std::list<int> > segment_similarity_list;
    utl::bronKerbosch (segmentSimilarityGraph, segment_similarity_list, 1);
    
    segment_similarity.resize(segment_similarity_list.size());
    int c = 0;
    for (std::list<std::list<int> >::iterator it1=segment_similarity_list.begin(); it1 != segment_similarity_list.end(); ++it1)
    {
      for (std::list<int>::iterator it2=it1->begin(); it2 != it1->end(); ++it2)
      {
        segment_similarity[c].push_back(*it2);
      }
      c++;
    }
  }  
  
  //----------------------------------------------------------------------------
  // Foreground segmentation
  //----------------------------------------------------------------------------  
  
  /** \brief Compute the cloud adjacency weights. This consists of two steps:
   *  1. Computing the adjacency between points of the cloud
   *  2. Caclulating the similarity weights between adjacent points. Similarity
   * is calculated based on the angle between the normals of the two adjacent
   * points.
   *  \param[in]  cloud             input pointcloud
   *  \param[in]  radius            maximum distance between adjacent points
   *  \param[in]  num_neighbors     maximum number of neighbors of a point
   *  \param[in]  sigma_convex      sigma for calculating similarity between two points in a convex arrangement
   *  \param[in]  sigma_concave     sigma for calculating similarity between two points in a concave arrangement
   *  \param[out] binary_weights    graph structure holding both adjacency information as well as adjacecy weights
   */  
  template <typename PointT>
  inline
  bool cloudAdjacencyWeights  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const pcl::search::KdTree<PointT> &search_tree,
                                const float radius,
                                const int   num_neighbors,
                                const float sigma_convex,
                                const float sigma_concave,
                                utl::GraphWeighted &graph_weighted
                              )
  {
    graph_weighted.clear();     // A graph defining the adjacency between pointcloud points
    
    // Get adjacency
    if (!utl::getCloudConnectivityRadius<PointT>(cloud, search_tree, graph_weighted, radius, num_neighbors))
      return false;
    
    // Calculate weights 
    for (size_t edgeId = 0; edgeId < graph_weighted.getNumEdges(); edgeId++)
    {
      int vtx1Id, vtx2Id;
      float weight;
      if (!graph_weighted.getEdge(edgeId, vtx1Id, vtx2Id, weight))
        return false;
            
      Eigen::Vector3f p1 = cloud->points[vtx1Id].getVector3fMap();
      Eigen::Vector3f p2 = cloud->points[vtx2Id].getVector3fMap();
      Eigen::Vector3f n1 = cloud->points[vtx1Id].getNormalVector3fMap();
      Eigen::Vector3f n2 = cloud->points[vtx2Id].getNormalVector3fMap();

      // Get angle per distance
      float dotProd = utl::clampValue(n1.dot(n2), -1.0f, 1.0f);   // Get the cosine of the angle between the normals
      dotProd = -dotProd + 1.0f;
      
      // Get dissimilarity measure
      float sigmaConvex   = 2;
      float sigmaConcave  = 0.15;
      
      float similarity;
      if (n1.dot(p1-p2) > 0)
        similarity = std::exp(- dotProd / sigmaConvex);
      else
        similarity = std::exp(- dotProd / sigmaConcave);
            
      if (!graph_weighted.setEdgeWeight(vtx1Id, vtx2Id, similarity))
        return false;
    }

    return true;
  }
  
  /** \brief Compute the cloud adjacency weights. This consists of two steps:
   *  1. Computing the adjacency between points of the cloud
   *  2. Caclulating the similarity weights between adjacent points. Similarity
   * is calculated based on the angle between the normals of the two adjacent
   * points.
   *  \param[in]  cloud             input pointcloud
   *  \param[in]  radius            maximum distance between adjacent points
   *  \param[in]  num_neighbors     maximum number of neighbors of a point
   *  \param[in]  sigma_convex      sigma for calculating similarity between two points in a convex arrangement
   *  \param[in]  sigma_concave     sigma for calculating similarity between two points in a concave arrangement
   *  \param[out] binary_weights    graph structure holding both adjacency information as well as adjacecy weights
   */  
  template <typename PointT>
  inline
  bool cloudAdjacencyWeights  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const float radius,
                                const int   num_neighbors,
                                const float sigma_convex,
                                const float sigma_concave,
                                utl::GraphWeighted &graph_weighted
                              )
  {
    graph_weighted.clear();     // A graph defining the adjacency between pointcloud points
    
    // Get adjacency
    utl::getCloudConnectivityRadius<PointT>(cloud, graph_weighted, radius, num_neighbors);
    
    // Calculate weights 
    for (size_t edgeId = 0; edgeId < graph_weighted.getNumEdges(); edgeId++)
    {
      int vtx1Id, vtx2Id;
      float weight;
      if (!graph_weighted.getEdge(edgeId, vtx1Id, vtx2Id, weight))
        return false;
            
      Eigen::Vector3f p1 = cloud->points[vtx1Id].getVector3fMap();
      Eigen::Vector3f p2 = cloud->points[vtx2Id].getVector3fMap();
      Eigen::Vector3f n1 = cloud->points[vtx1Id].getNormalVector3fMap();
      Eigen::Vector3f n2 = cloud->points[vtx2Id].getNormalVector3fMap();

      // Get angle per distance
      float dotProd = utl::clampValue(n1.dot(n2), -1.0f, 1.0f);   // Get the cosine of the angle between the normals
      dotProd = -dotProd + 1.0f;
      
      // Get dissimilarity measure
      float sigmaConvex   = 2;
      float sigmaConcave  = 0.15;
      
      float similarity;
      if (n1.dot(p1-p2) > 0)
        similarity = std::exp(- dotProd / sigmaConvex);
      else
        similarity = std::exp(- dotProd / sigmaConcave);
            
      if (!graph_weighted.setEdgeWeight(vtx1Id, vtx2Id, similarity))
        return false;
    }

    return true;
  }
  
  inline
  bool segmentCloudFG ( const std::vector<float> &fg_weights,
                        const std::vector<float> &bg_weights,
                        const utl::GraphWeighted &edge_weights,
                        const utl::Map &downsample_map,
                        std::vector<int> &fg_points,
                        std::vector<int> &fg_points_full_res
                      )
  {
    // Segment
    std::vector<int> bgPoints;
    if (utl::mincut(fg_weights, bg_weights, edge_weights, bgPoints, fg_points) < 0.0f)
    {
      return false;
    }
    
    // Get high res object segments
    for (size_t lrPtId = 0; lrPtId < fg_points.size(); lrPtId++)
      fg_points_full_res.insert(fg_points_full_res.begin(), downsample_map[fg_points[lrPtId]].begin(), downsample_map[fg_points[lrPtId]].end());
    
    return true;
  }
}

#endif    // SEGMENTATION_HPP
