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

#ifndef SCENE_OVERSEGMENTATION_HPP
#define SCENE_OVERSEGMENTATION_HPP

// // PCL includes
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>

// Segmentation includes
#include <region_growing_smoothness/region_growing_smoothness.hpp>
#include <segmentation.hpp>


template <typename PointT>
bool oversegmentScene ( const typename pcl::PointCloud<PointT>::ConstPtr  &scene_cloud,
                        const utl::SmoothSegParams                        &overseg_params,
                        typename pcl::PointCloud<PointT>::Ptr             &scene_cloud_ds,
                        utl::Map                                          &downsample_map,
                        std::vector<utl::Map>                             &overseg_segments,
                        utl::Map                                          &overseg_segments_linear,
                        bool                                              consistent_normals = true
                      )
{
  scene_cloud_ds.reset(new pcl::PointCloud<PointT>);
  
  if (scene_cloud->size() < 3)
    return true;
  
  //----------------------------------------------------------------------------
  // Find smooth segments in the cloud
  //----------------------------------------------------------------------------

  // Downsample the cloud
  if (overseg_params.voxel_size <= 0.0f)
  {
    std::cout << "Segmentation voxel size must be greater than 0." << std::endl;
    std::cout << "Current segmentation voxel size is ." << overseg_params.voxel_size << " " << std::endl;
    return false;
  }
  else
  {
    utl::Downsample<PointT> ds;
    ds.setInputCloud(scene_cloud);
    ds.setDownsampleMethod(utl::Downsample<PointT>::AVERAGE);
    ds.setLeafSize(overseg_params.voxel_size);
    ds.filter(*scene_cloud_ds);
    ds.getDownsampleMap (downsample_map);
  }
  
  // Segment
  int numSmoothThresholds = overseg_params.smoothness.size();
  std::vector<utl::Map> oversegSegmentsRaw(numSmoothThresholds);
  
  #pragma omp parallel for
  for (size_t segParamId = 0; segParamId < numSmoothThresholds; segParamId++)
  {
    utl::RegionGrowingSmoothness<PointT> rg;
    rg.setInputCloud(scene_cloud_ds);
    rg.setConsistentNormals(consistent_normals);
    rg.setSearchRadius (overseg_params.voxel_size * std::sqrt (3));
    rg.setMinSegmentSize(overseg_params.min_segment_size);
    float normalVariation   = overseg_params.smoothness[segParamId].first;
    float validBinFraction  = overseg_params.smoothness[segParamId].second;
    rg.setNormalAngleThreshold(normalVariation);
    rg.setMinValidBinaryNeighborsFraction(validBinFraction);
    rg.segment(oversegSegmentsRaw[segParamId]);
  }
  
  // Merge similar segments
  std::vector<std::vector<int> > seg_merged_ids (numSmoothThresholds);
  utl::mergeDuplicateSegments  (oversegSegmentsRaw, seg_merged_ids, overseg_params.max_iou);

  overseg_segments.resize(numSmoothThresholds);
  for (size_t segParamId = 0; segParamId < numSmoothThresholds; segParamId++)
  {
    for (size_t segIdIt = 0; segIdIt < seg_merged_ids[segParamId].size(); segIdIt++)
    {
      int segId = seg_merged_ids[segParamId][segIdIt];
      overseg_segments[segParamId].push_back(oversegSegmentsRaw[segParamId][segId]);
      overseg_segments_linear.push_back(oversegSegmentsRaw[segParamId][segId]);
    }
  }
}

#endif     // SCENE_OVERSEGMENTATION_HPP