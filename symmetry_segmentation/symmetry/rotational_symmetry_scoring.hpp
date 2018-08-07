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

#ifndef ROTATIONAL_SYMMETRY_SCORING_HPP
#define ROTATIONAL_SYMMETRY_SCORING_HPP

// Octomap includes
#include <occupancy_map.hpp>

// Symmetry
#include <symmetry/rotational_symmetry.hpp>

namespace sym
{
  /** \brief Calculate how well does a rotational symmetry fit a pointcloud.
   *  \param[in]  cloud                   input cloud
   *  \param[in]  symmetry                input symmetry
   *  \param[in]  point_symmetry_scores   symmetry scores for individual points
   *  \param[in]  min_normal_fit_angle    minimum fit angle between a symmetry and a point
   *  \param[in]  max_normal_fit_angle    maximum fit angle between a symmetry and a point
   *  \return symmetry score for the whole pointcloud
   */
  template <typename PointT>
  inline
  float rotSymCloudSymmetryScore  ( const pcl::PointCloud<PointT> &cloud,
                                    const sym::RotationalSymmetry &symmetry,
                                    std::vector<float> &point_symmetry_scores,
                                    const float min_normal_fit_angle = 0.0f,
                                    const float max_normal_fit_angle = M_PI / 2
                                  )
  {
    point_symmetry_scores.resize(cloud.size());
        
    // Get point symmetry scores
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
    {
      Eigen::Vector3f point   = cloud.points[pointId].getVector3fMap();
      Eigen::Vector3f normal  = cloud.points[pointId].getNormalVector3fMap();
      float angle = getRotSymFitError  (point, normal, symmetry);
      float score = (angle - min_normal_fit_angle) / (max_normal_fit_angle - min_normal_fit_angle);
      score = utl::clampValue(score, 0.0f, 1.0f);
      point_symmetry_scores[pointId] = score;
    }
    
    // Normalize and return
    return utl::mean(point_symmetry_scores);
  }
  
  /** \brief Calculate the occlusion score for a pointcloud and a symmetry.
   *  \param[in]  cloud                   input cloud
   *  \param[in]  occupancy_map           occupancy map of the scene
   *  \param[in]  symmetry                input symmetry
   *  \param[in]  point_occlusion_scores  occlusion scores for individual points
   *  \param[in]  min_occlusion_distance  minimum distance between a point and occluded/occupied space
   *  \param[in]  max_occlusion_distance  maximum distance between a point and occluded/occupied space
   *  \param[in]  num_divisions           number of rotations of a pointcloud
   *  \return occlusion score of the whole pointcloud
   */
  template <typename PointT>
  inline
  float rotSymCloudOcclusionScore ( const pcl::PointCloud<PointT> &cloud,
                                    const OccupancyMapConstPtr &occupancy_map,
                                    const sym::RotationalSymmetry &symmetry,
                                    std::vector<float> &point_occlusion_scores,
                                    const float min_occlusion_distance = 0.0f,
                                    const float max_occlusion_distance = 1.0f,
                                    const int num_divisions = 12
                                  )
  {
    point_occlusion_scores.resize(cloud.size());
    
    // Reconstruct pointcloud
    typename pcl::PointCloud<PointT>::Ptr cloudReconstructed (new pcl::PointCloud<PointT>);
    symmetry.reconstructCloud<PointT>(cloud, *cloudReconstructed, 2.0f * M_PI / static_cast<float> (num_divisions));
    
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
    {
      float maxDistance = 0.0f;
      
      for (size_t divId = 0; divId < num_divisions; divId++)
      {
        int recPointId = (divId * cloud.size()) + pointId;
        Eigen::Vector3f recPoint = cloudReconstructed->points[recPointId].getVector3fMap();        
        
        // Get distance from reconstrcted point to occluded/occupied space
        float curDistance = occupancy_map->getNearestObstacleDistance(recPoint);
        maxDistance = std::max(maxDistance, curDistance);
        
        if (maxDistance >= max_occlusion_distance)
          break;        
      }
      
      float score = (maxDistance - min_occlusion_distance) / (max_occlusion_distance - min_occlusion_distance);      
      score = utl::clampValue(score, 0.0f, 1.0f);
      point_occlusion_scores[pointId] = score;
    }
    
    // Normalize and return
    return utl::mean(point_occlusion_scores);    
  }  
  
  /** \brief Calculate how perpendicular a pointcloud is to the symmetry axis.
   * The final score is in the [0, 1] range. Higher values indicate higher
   * perpendicularity.
   *  \param[in]  cloud                       input cloud
   *  \param[in]  symmetry                    input symmetry
   *  \param[in]  point_perpendicular_scores  perpendicularity scores for individual points
   *  \param[in]  angle_threshold angle threshold used for clamping
   *  \return cloud perpendicularity score
   */
  template <typename PointT>
  inline
  float rotSymCloudPerpendicularScores  ( const pcl::PointCloud<PointT> &cloud,
                                          const sym::RotationalSymmetry &symmetry,
                                          std::vector<float> &point_perpendicular_scores,
                                          const float angle_threshold = M_PI / 2
                                        )
  {
    // Check that cloud is non-zero
    if (cloud.size() == 0)
    {
      std::cout << "[sym::rotSymCloudPerpendicularScores] input cloud is empty!" << std::endl;
      return -1.0f;
    }
    
    // Calculate perpendicularity
    point_perpendicular_scores.resize(cloud.size());
    
    for (size_t pointId = 0; pointId < cloud.size (); pointId++)
      point_perpendicular_scores[pointId] = sym::getRotSymPerpendicularity(cloud.points[pointId].getNormalVector3fMap (), symmetry, angle_threshold);
    
    // Normalize and return
    return utl::mean(point_perpendicular_scores);
  }
  
  /** \brief Get the angle measuring how much the pointcloud "wraps" around 
   * the symmetry axis. It is calculated as 2*pi - the maximum angular step
   * between adjacent points of the pointcloud. The maximum angular step is
   * computed as:
   * 1. For each point find the vector that goes from point projected on the 
   *    symmetry axis to the point itself.
   * 2. Choose a random vector.
   * 3. Compute the clockwise angles between the random vector and all other
   *    vectors.
   * 4. Sort the angles in increasing order.
   * 5. Find the largest step between two adjacent angles.
   *  \param[in]  cloud                   input cloud
   *  \param[in]  symmetry                input symmetry
   *  \return largest angular step
   */
  template <typename PointT>
  float rotSymCloudCoverageAngle (const pcl::PointCloud<PointT> &cloud, const sym::RotationalSymmetry &symmetry)
  {
    // If cloud is empty - make a warning and return -1.
    if (cloud.empty ())
    {
      std::cout << "[sym::rotSymCoverageAngle] pointcloud is empty! Returning -1." << std::endl;
      return -1.0f;
    }
    
    // If there is a single point - return 360 degrees.
    if (cloud.size () == 1)
    {
      return 0.0f;
    }      
    
    // Get reference vector
    Eigen::Vector3f referenceVector = cloud.points[0].getVector3fMap () - symmetry.projectPoint(cloud.points[0].getVector3fMap ());
    
    // Find angles between vectors formed by all other points and current vector.
    std::vector<float> angles (cloud.size ());
    angles[0] = 0.0f;
    for (size_t pointId = 1; pointId < cloud.size (); pointId++)
    {
      Eigen::Vector3f curVector = cloud.points[pointId].getVector3fMap () - symmetry.projectPoint(cloud.points[pointId].getVector3fMap ());
      angles[pointId] = utl::vectorVectorAngleCW<float>(referenceVector, curVector, symmetry.getDirection ());
    }
      
    // Sort the angles
    std::sort(angles.begin(), angles.end());
    
    // Get angle difference
    std::vector<float> angleDifference(angles.size());
    for (size_t i = 1; i < angles.size(); i++)
      angleDifference[i] = utl::angleDifferenceCCW(angles[i-1], angles[i]);
    angleDifference[0] = utl::angleDifferenceCCW(angles[angles.size()-1], angles[0]);
    
    return (2.0f * M_PI) - utl::vectorMax(angleDifference);
  }
}

#endif    // ROTATIONAL_SYMMETRY_SCORING_HPP