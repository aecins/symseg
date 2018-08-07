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

#ifndef REFLECTIONAL_SYMMETRY_SEGMENTATION_H
#define REFLECTIONAL_SYMMETRY_SEGMENTATION_H

// Symmetry includes
#include <symmetry/reflectional_symmetry.hpp>
#include <occupancy_map.hpp>

namespace sym
{
  //----------------------------------------------------------------------------
  // Segmentation parameters
  //----------------------------------------------------------------------------
  
  struct ReflSymSegParams
  {
    // Downsample parameters
    float voxel_size = 0.01f;
    
    // Point symmetry scoring parameters
    float max_sym_corresp_reflected_distance = 0.01f;
    float min_occlusion_distance = 0.005f;
    float max_occlusion_distance = 0.03f;
    float min_normal_fit_angle = pcl::deg2rad(10.0f);
    float max_normal_fit_angle = pcl::deg2rad(45.0f);
    
    // Cloud adjacency weights parameters
    float aw_radius = std::max(voxel_size, 0.005f) * 2.0f;
    int   aw_num_neighbors = 9;                   // NOTE: it is very important to have sufficient connectivity between points. High number of neighbors is encouraged
    float aw_sigma_convex = 2.0f;
    float aw_sigma_concave = 0.15f;
    
    // Adjacency weight importance
    float adjacency_importance = 1.0f;
    float symmetric_adjacency_importance = 1.0f;
    
      // Segmentation parameters
    float fg_weight_importance    = 1.0f;
    float bg_weight_importance    = 1.0f;
    float bin_weight_importance  = 10.0f;
    float min_occlusion_score     = 0.1f;
    
    // Segmentation filtering parameters
    float max_symmetry_score    = 0.005f;
    float max_occlusion_score   = 0.0005f;
    float max_smoothness_score  = 0.2f;
    int   min_segment_size      = 100;
    int   min_symmetry_support_overlap = 0.5f;
    
    // Segmentation merge parameters
    float similar_segment_iou_ = 0.95f;
  };

  //----------------------------------------------------------------------------
  // Symmetry detection class
  //----------------------------------------------------------------------------

  /** \brief Rotational symmmetry detection. */
  template <typename PointT>
  class ReflectionalSymmetrySegmentation
  {
  public:
    
    /** \brief Empty constructor. */
    ReflectionalSymmetrySegmentation ();
    
    /** \brief Constructor with custom parameters. */
    ReflectionalSymmetrySegmentation (const ReflSymSegParams &params);
    
    /** \brief Destructor. */
    ~ReflectionalSymmetrySegmentation ();
    
    /** \brief Provide a pointer to the input pointcloud.
     *  \param cloud the const boost shared pointer to a pointcloud
     */
    inline
    void setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);
    
    /** \brief Provide a pointer to the input scene occupancy map.
     *  \param occupancy_map the const boost shared pointer to an occupancy map
     */
    inline
    void setInputOcuppancyMap  (const OccupancyMapConstPtr &occupancy_map);

    /** \brief Set the table plane of the scene
     *  \param table_plane coefficients of the table plane
     */
    inline
    void setInputTablePlane (const Eigen::Vector4f &table_plane);
    
    /** \brief Provide a pointer to the input scene occupancy map.
     *  \param cloud the const boost shared pointer to an occupancy map
     */
    inline
    void setInputSymmetries  (const std::vector<sym::ReflectionalSymmetry> &symmetries, const std::vector<std::vector<int> > &symmetry_support);
    
    /** \brief Set detection parameters.
     *  \param params detection parameters
     */
    inline
    void setParameters (const ReflSymSegParams &params);
    
    /** \brief Segment rotational symmetries. */
    inline bool segment ();

    /** \brief Filter detected segments. */
    inline void filter ();
    
    /** \brief Filter detected symmetries. */
    inline void merge ();
    
    /** \brief Get segments
     *  \param[out] segments    segments over the original cloud
     */
    inline
    void getSegments  ( std::vector<std::vector<int> >  &segments, std::vector<int> &segment_filtered_ids, std::vector<std::vector<int> > &segment_merged_ids);
    
    /** \brief Get segments calculated for the downsampled cloud.
     *  \param[out] cloud_ds      downsampled cloud
     *  \param[out] segments_ds   segments over downsampled cloud
     */
    inline
    void getSegmentsDownsampled ( typename pcl::PointCloud<PointT>::Ptr &cloud_ds,
                                  std::vector<std::vector<int> >        &segments_ds  );
 
    /** \brief Get scores for the refined symmetries.
     *  \param[out] symmetry_scores     symmetry scores
     *  \param[out] occlusion_scores    symmetry occlusion scores
     *  \param[out] smoothness_scores   smoothness scores
     */
    inline
    void getScores  ( std::vector<float> &symmetry_scores,
                      std::vector<float> &occlusion_scores,
                      std::vector<float> &smoothness_scores );
    
    /** \brief Get point scores.
     *  \param[out] point_symmetry_scores       symmetry scores
     *  \param[out] point_occlusion_scores      occlusion scores
     *  \param[out] point_perpendicular_scores  occlusion scores
     *  \param[out] correspondences             symmetric correspondences
     */
    inline
    void getPointScores ( std::vector<std::vector<float> >  &point_symmetry_scores,
                          std::vector<std::vector<float> >  &point_occlusion_scores,
                          std::vector<std::vector<float> >  &point_perpendicular_scores,
                          std::vector<pcl::Correspondences> &correspondences );

    /** \brief Get adjacency weights.
     *  \param[out] adjacency   pointcloud adjacency
     */
    inline
    void getAdjacency ( utl::GraphWeighted &adjacency, std::vector<utl::GraphWeighted> &symmetric_adjacency  );
    
    /** \brief Get weights used in the graph cut.
     *  \param[out] fg_weights      foreground weights
     *  \param[out] bg_weights      background weights
     *  \param[out] binary_weights  binary weights
     */
    inline
    void getGraphCutWeights ( std::vector<std::vector<float> >        &fg_weights,
                              std::vector<std::vector<float> >        &bg_weights,
                              std::vector<utl::GraphWeighted>  &binary_weights,
                              std::vector<utl::GraphWeighted>  &cut_edges  );
    
  private:
    
    /** \brief Detection parameters. */
    ReflSymSegParams params_;
    
    /** \brief Input cloud. */
    typename pcl::PointCloud<PointT>::ConstPtr cloud_;
        
    /** \brief Downsampled input cloud. */
    typename pcl::PointCloud<PointT>::Ptr cloud_ds_;

    /** \brief Search tree over the downsampled input cloud. */
    pcl::search::KdTree<PointT> cloud_search_tree_;
    
    /** \brief Scene occupancy map. */
    OccupancyMapConstPtr occupancy_map_;
    
    /** \brief Table plane. */
    Eigen::Vector4f table_plane_;

    /** \brief Downsample map for the downsample pointcloud. */
    utl::Map downsample_map_;
        
    /** \brief Input symmetries. */
    std::vector<sym::ReflectionalSymmetry> symmetries_;

    /** \brief Segments for the input symmetries. */
    std::vector<std::vector<int> > symmetry_support_segments_;

    /** \brief Downsampled segments for the input symmetries. */
    std::vector<std::vector<int> > symmetry_support_segments_ds_;
    
    /** \brief Segments. */
    std::vector<std::vector<int> > segments_;
    
    /** \brief Downsampled segments . */
    std::vector<std::vector<int> > segments_ds_;
    
    /** \brief Point symmetry scores. */
    std::vector<std::vector<float> >  point_symmetry_scores_;
    
    /** \brief Point occlusion scores. */
    std::vector<std::vector<float> >  point_occlusion_scores_;
    
    /** \brief Point occlusion scores. */
    std::vector<std::vector<float> >  point_perpendicular_scores_;
    
    /** \brief Correspondences. */
    std::vector<pcl::Correspondences> correspondences_;
        
    /** \brief Adjacency weights. */
    utl::GraphWeighted adjacency_;
    
    /** \brief Adjacency weights. */
    std::vector<utl::GraphWeighted> symmetric_adjacency_;

    /** \brief Foreground weights. */
    std::vector<std::vector<float> > fg_weights_;
    
    /** \brief Background weights. */
    std::vector<std::vector<float> > bg_weights_;
    
    /** \brief Binary weights. */
    std::vector<utl::GraphWeighted> binary_weights_;
    
    /** \brief Edges of the cut induced by the segmentation. */
    std::vector<utl::GraphWeighted> cut_edges_;
    
    /** \brief Segment symmetry scores. */
    std::vector<float> symmetry_scores_;
        
    /** \brief Segment occlusion scores. */
    std::vector<float> occlusion_scores_;

    /** \brief Segment smoothness scores. */
    std::vector<float> smoothness_scores_;
    
    /** \brief Symmetry support overlap scores. */
    std::vector<float> symmetry_support_overlap_scores_;

    /** \brief Indices of the filtered segments. */
    std::vector<int> segment_filtered_ids_;
    
    /** \brief Merged filtered segments. */
    std::vector<std::vector<int> > segment_merged_ids_;
  };
}

#endif // REFLECTIONAL_SYMMETRY_SEGMENTATION_H