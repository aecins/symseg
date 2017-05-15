// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef ROTATIONAL_SYMMETRY_DETECTION_H
#define ROTATIONAL_SYMMETRY_DETECTION_H

#include <symmetry/rotational_symmetry.hpp>
#include <occupancy_map.hpp>

namespace sym
{
  //----------------------------------------------------------------------------
  // Symmetry detection parameters
  //----------------------------------------------------------------------------

  struct RotSymDetectParams
  {
    // Refinement
    float ref_max_fit_angle         = pcl::deg2rad (45.0f);  
  
    // Scoring
    float min_normal_fit_angle    = pcl::deg2rad(10.0f);
    float max_normal_fit_angle    = pcl::deg2rad(60.0f);
    float min_occlusion_distance  = 0.01f;
    float max_occlusion_distance  = 0.03f;
    
    // Filtering
    float max_symmetry_score      = 0.01f;
    float max_occlusion_score     = 0.01f;
    float max_perpendicular_score = 0.65f;
    float min_coverage_score      = 0.3f;
  };
  
  //----------------------------------------------------------------------------
  // Symmetry detection merging
  //----------------------------------------------------------------------------
  
  /** \brief Merge symemtry hypotheses that are similar enough.
   *  \param[in]  symmetries                  input symmetries
   *  \param[in]  symmetry_reference_points   reference points for input symmetries
   *  \param[in]  indices                     indices of the symmetries used
   *  \param[in]  symmetry_scores             symmetry scores for input symmetries
   *  \param[in]  occlusion_scores            occlusion scores for input symmetries
   *  \param[out] merged_sym_ids              ids of symmetries after merging
   *  \param[in]  max_angle_diff              maximum angle between the axes of two distinct symmetries that will be merged
   *  \param[in]  max_distance_diff           minimum distance between two symmetries that will be merged
   */
  inline
  void mergeDuplicateRotSymmetries  ( const std::vector<sym::RotationalSymmetry> &symmetries,
                                      const std::vector<Eigen::Vector3f> &symmetry_reference_points,
                                      const std::vector<int> &indices,
                                      const std::vector<float> &support_size,
                                      const std::vector<float> &occlusion_scores,
                                      std::vector<int> &merged_sym_ids,
                                      const float max_angle_diff = pcl::deg2rad(10.0f),
                                      const float max_distance_diff = 0.01f
                                    );

  /** \brief Merge symemtry hypotheses that are similar enough.
   *  \param[in]  symmetries                  input symmetries
   *  \param[in]  symmetry_reference_points   reference points for input symmetries
   *  \param[in]  indices                     indices of the symmetries used
   *  \param[in]  symmetry_scores             symmetry scores for input symmetries
   *  \param[in]  occlusion_scores            occlusion scores for input symmetries
   *  \param[out] merged_sym_ids              ids of symmetries after merging
   *  \param[in]  max_angle_diff              maximum angle between the axes of two distinct symmetries that will be merged
   *  \param[in]  max_distance_diff           minimum distance between two symmetries that will be merged
   */
  inline
  void mergeDuplicateRotSymmetries  ( const std::vector<sym::RotationalSymmetry> &symmetries,
                                      const std::vector<Eigen::Vector3f> &symmetry_reference_points,
                                      const std::vector<float> &support_size,
                                      const std::vector<float> &occlusion_scores,
                                      std::vector<int> &merged_sym_ids,
                                      const float max_normal_angle_diff = pcl::deg2rad(10.0f),
                                      const float max_distance_diff = 0.01f
                                    );
  
  //----------------------------------------------------------------------------
  // Symmetry detection class
  //----------------------------------------------------------------------------

  /** \brief Rotational symmmetry detection. */
  template <typename PointT>
  class RotationalSymmetryDetection
  {
  public:
    
    /** \brief Empty constructor. */
    RotationalSymmetryDetection ();
    
    /** \brief Constructor with custom parameters. */
    RotationalSymmetryDetection (const RotSymDetectParams &params);
    
    /** \brief Destructor. */
    ~RotationalSymmetryDetection ();
    
    /** \brief Provide a pointer to the input pointcloud.
     *  \param cloud pointer to a pointcloud
     */
    inline
    void setInputCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud);
    
    /** \brief Provide a pointer to the input scene occupancy map.
     *  \param occupancy_map pointer to an occupancy map
     */
    inline
    void setInputOcuppancyMap  (const OccupancyMapConstPtr &occupancy_map);
    
    /** \brief Set initial symmetries.
     *  \param initial_symmetries   vector of initial symmetries
     */
    inline
    void setInputSymmetries  (const std::vector<sym::RotationalSymmetry> &symmetries_initial);
    
    /** \brief Set detection parameters.
     *  \param params detection parameters
     */
    inline
    void setParameters (const RotSymDetectParams &params);
    
    /** \brief Detect reflectional symmetries in the input pointcloud. */
    inline bool detect ();

    /** \brief Filter detected symmetries. */
    inline void filter ();
    
    /** \brief Filter detected symmetries. */
    inline void merge ();
    
    /** \brief Get all of the refined symmetries as well as indices of filtered
     * and merged symmetries.
     *  \param[out] symmetries              vector of all refined symmetries
     *  \param[out] symmetry_filtered_ids   indices of filtered symmetries
     *  \param[out] symmetry_merged_ids     indices of merged symmetries
     */
    inline
    void getSymmetries  ( std::vector<sym::RotationalSymmetry> &symmetries,
                          std::vector<int> &symmetry_filtered_ids,
                          std::vector<int> &symmetry_merged_ids );
 
    /** \brief Get scores for the refined symmetries.
     *  \param[out] symmetry_scores           symmetry scores
     *  \param[out] occlusion_scores_         symmetry occlusion scores
     *  \param[out] perpendicular_scores_     perpendicular scores
     *  \param[out] coverage_scores_          coverage scores
     */
    inline
    void getScores  ( std::vector<float> &symmetry_scores,
                      std::vector<float> &occlusion_scores,
                      std::vector<float> &perpendicular_scores,
                      std::vector<float> &coverage_scores  );
    
    /** \brief Get point symmetry and occlusion scores for the refined symmetries.
     *  \param[out] point_symmetry_scores       symmetry scores
     *  \param[out] point_occlusion_scores      occlusion scores
     *  \param[out] point_perpendicular_scores  perpendicular scores
     */
    inline
    void getPointScores ( std::vector<std::vector<float> >  &point_symmetry_scores,
                          std::vector<std::vector<float> >  &point_occlusion_scores,
                          std::vector<std::vector<float> >  &point_perpendicular_scores,
                          std::vector<int>                  &cloud_no_boundary_point_ids );
    
  private:
    
    /** \brief Detection parameters. */
    RotSymDetectParams params_;
    
    /** \brief Input cloud. */
    typename pcl::PointCloud<PointT>::ConstPtr cloud_;

    /** \brief Input cloud with boundary points removed. */
    typename pcl::PointCloud<PointT>::Ptr cloud_no_boundary_;
    
    /** \brief Indices of non-boundary points of the cloud. */
    std::vector<int> cloud_no_boundary_point_ids_;
    
    /** \brief Input cloud. */
    Eigen::Vector3f cloud_mean_;
    
    /** \brief Scene occupancy map. */
    OccupancyMapConstPtr occupancy_map_;

    /** \brief Refined symmetries. */
    std::vector<sym::RotationalSymmetry> symmetries_initial_;
    
    /** \brief Refined symmetries. */
    std::vector<sym::RotationalSymmetry> symmetries_refined_;

    /** \brief Symmetry scores. */
    std::vector<float> symmetry_scores_;
        
    /** \brief Symmetry scores. */
    std::vector<float> occlusion_scores_;

    /** \brief Symmetry inlier scores. */
    std::vector<float> perpendicular_scores_;
    
    /** \brief Symmetry scores. */
    std::vector<float> coverage_scores_;

    /** \brief Pointwise symmetry scores */
    std::vector<std::vector<float> > point_symmetry_scores_;
    
    /** \brief Pointwise occlusion scores */
    std::vector<std::vector<float> > point_occlusion_scores_;
    
    /** \brief Pointwise occlusion scores */
    std::vector<std::vector<float> > point_perpendicular_scores_;
    
    /** \brief Indices of the filtered symmetries. */
    std::vector<int> symmetry_filtered_ids_;
    
    /** \brief Indices of the merged symmetries. */
    std::vector<int> symmetry_merged_ids_;
  };
}

#endif  // ROTATIONAL_SYMMETRY_DETECTION_H