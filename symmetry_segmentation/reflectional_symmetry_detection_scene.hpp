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

#ifndef REFLECTIONAL_SYMMETRY_DETECTION_SCENE_HPP
#define REFLECTIONAL_SYMMETRY_DETECTION_SCENE_HPP

// Symmetry includes
#include <symmetry/reflectional_symmetry_detection.hpp>

template <typename PointT>
bool detectReflectionalSymmetryScene  ( const typename pcl::PointCloud<PointT>::ConstPtr  &scene_cloud,
                                        const OccupancyMapConstPtr                        &scene_occupancy_map,
                                        const utl::Map                                    &segments,
                                        const sym::ReflSymDetectParams                    &sym_detect_params,
                                        std::vector<sym::ReflectionalSymmetry>            &symmetry,
                                        std::vector<std::vector<int> >                    &symmetry_support_segments
                                      )
{
  symmetry.resize(0);
  symmetry_support_segments.resize(0);
  
  if (scene_cloud->size() < 3)
    return true;

  //----------------------------------------------------------------------------
  // Reflectional symmetry detection
  //----------------------------------------------------------------------------
  
  std::vector<typename pcl::PointCloud<PointT>::Ptr>    segmentClouds                 (segments.size());
  std::vector<std::vector<sym::ReflectionalSymmetry> >  symmetry_TMP                  (segments.size());
  std::vector<std::vector<int> >                        symmetryFilteredIds_TMP       (segments.size());
  std::vector<std::vector<int> >                        symmetryMergedIds_TMP         (segments.size());
  std::vector<std::vector<float> >                      occlusionScores_TMP           (segments.size());
  std::vector<std::vector<float> >                      cloudInlierScores_TMP         (segments.size());  
  std::vector<std::vector<float> >                      correspInlierScores_TMP       (segments.size());
  
  # pragma omp parallel for
  for (size_t segId = 0; segId < segments.size(); segId++)
  {
    segmentClouds[segId].reset(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*scene_cloud, segments[segId], *segmentClouds[segId]);
    
    sym::ReflectionalSymmetryDetection<PointT> rsd (sym_detect_params);
    rsd.setInputCloud(segmentClouds[segId]);
    rsd.setInputOcuppancyMap(scene_occupancy_map);
    rsd.detect();
    rsd.filter();
    rsd.merge();
    rsd.getSymmetries(symmetry_TMP[segId], symmetryFilteredIds_TMP[segId], symmetryMergedIds_TMP[segId]);
    rsd.getScores(occlusionScores_TMP[segId], cloudInlierScores_TMP[segId], correspInlierScores_TMP[segId]);
  }

  //----------------------------------------------------------------------------
  // Merge symmetries for the whole cloud
  //----------------------------------------------------------------------------
  
  // Linearize symmetry data
  std::vector<sym::ReflectionalSymmetry>  symmetry_linear;
  std::vector<std::pair<int,int> >        symmetry_linearMap;    // A map from linear ID to corresponding segment and symmetry IDs
  std::vector<float>                      supportSizes_linear;
  std::vector<float>                      occlusionScores_linear;
  
  std::vector<Eigen::Vector3f> referencePoints_linear;
  for (size_t segId = 0; segId < symmetryMergedIds_TMP.size(); segId++)
  {
    for (size_t symIdIt = 0; symIdIt < symmetryMergedIds_TMP[segId].size(); symIdIt++)
    {
      int symId = symmetryMergedIds_TMP[segId][symIdIt];
      symmetry_linear.push_back(symmetry_TMP[segId][symId]);
      symmetry_linearMap.push_back(std::pair<int,int>(segId, symId));
      
      occlusionScores_linear.push_back(occlusionScores_TMP[segId][symId]);
      supportSizes_linear.push_back(static_cast<float>(segmentClouds[segId]->size()));
      
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*segmentClouds[segId], centroid);
      referencePoints_linear.push_back(centroid.head(3));
    }
  }
  
  // Merge similar symmetries
  std::vector<int> symmetryMergedGlobalIds_linear;
  sym::mergeDuplicateReflSymmetries ( symmetry_linear,
                                      referencePoints_linear,
                                      occlusionScores_linear,
                                      symmetryMergedGlobalIds_linear,
                                      sym_detect_params.symmetry_min_angle_diff,
                                      sym_detect_params.symmetry_min_distance_diff,
                                      sym_detect_params.max_reference_point_distance
                                    );  
  
  //----------------------------------------------------------------------------  
  // Construct final output
  //----------------------------------------------------------------------------
  
  symmetry.resize(symmetryMergedGlobalIds_linear.size());
  symmetry_support_segments.resize(symmetryMergedGlobalIds_linear.size());
  
  for (size_t symIdIt = 0; symIdIt < symmetryMergedGlobalIds_linear.size(); symIdIt++)
  {
    int symLinId  = symmetryMergedGlobalIds_linear[symIdIt];
    int segId     = symmetry_linearMap[symLinId].first;
    int symId     = symmetry_linearMap[symLinId].second;
    
    symmetry[symIdIt] = symmetry_TMP[segId][symId];
    symmetry_support_segments[symIdIt] = segments[segId];
  }
  
  return true;
}


#endif     // REFLECTIONAL_SYMMETRY_DETECTION_SCENE_HPP