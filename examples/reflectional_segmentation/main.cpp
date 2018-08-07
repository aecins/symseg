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

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>

// Utilities includes
#include "filesystem/filesystem.hpp"
#include "eigen.hpp"

// Occupancy map
#include "occupancy_map.hpp"

// Symmetry segmentation
#include "symmetry/reflectional_symmetry_segmentation.hpp"

#include "scene_oversegmentation.hpp"
#include "rotational_symmetry_detection_scene.hpp"
#include "reflectional_symmetry_detection_scene.hpp"

// Project includes
#include "vis.hpp"

typedef pcl::PointXYZRGBNormal PointNC;

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char** argv, std::string &inputPath, std::string &outputDirname, bool &visualize, bool &save)
{
  inputPath = "";
  outputDirname = "";
  visualize = true;
  save = false;
  
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
        
    if (curParameter == "-novis")
      visualize = false;   
    
    else if (curParameter == "-save")
      save = true;
        
    else if (inputPath == "" && curParameter[0] != '-')
      inputPath = curParameter;

    else if (outputDirname == "" && curParameter[0] != '-')
      outputDirname = curParameter;
    
    else 
      std::cout << "Unknown parameter '" << curParameter << "'" << std::endl;
  }  
}

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{  
  //----------------------------------------------------------------------------
  // Parse command line
  //----------------------------------------------------------------------------
  
  std::string inputPath, outputDirname;
  bool visualize, save;
  parseCommandLine(argc, argv, inputPath, outputDirname, visualize, save);
  
  if (outputDirname == "")
    outputDirname = "default";

  //----------------------------------------------------------------------------
  // Generate paths and check command line arguments
  //----------------------------------------------------------------------------
  
  std::string sceneDirname, sceneCloudFilename;
  
  if (inputPath == "")
  {
    std::cout << "You must provide an input path corresponding to a  pointcloud filename or a scene directory containing the pointcloud." << std::endl;
  }
  
  // Get scene directory
  if (utl::isDirectory(inputPath))
  {
//    std::cout << "Input path is a directory" << std::endl;
    sceneDirname = inputPath;
  }
  else if (utl::isFile(inputPath))
  {
//    std::cout << "Input path is a file" << std::endl;
    sceneDirname = utl::getParentDir(inputPath);
  }
  else
  {
    std::cout << "Could not find a directory or file with path '" << inputPath << "'." << std::endl;
  }
  
  // Get scene cloud filename
  sceneCloudFilename = utl::fullfile(sceneDirname, "cloud.ply");
  std::cout << "Scene directory: " << sceneDirname << std::endl;
//  std::cout << "Pointcloud file: " << sceneCloudFilename << std::endl;
  
  if (!utl::isFile(sceneCloudFilename))
  {
    std::cout << "Couldn't find pointcloud file '" << sceneCloudFilename << "'" << std::endl;
    return -1;
  }

  std::string octomapFilename             = utl::fullfile(sceneDirname, "occupancy.bt");  
  std::string tablePlaneFilename          = utl::fullfile(sceneDirname, "table_plane.txt");
  std::string symmetrySegmentationDirname = utl::fullfile(sceneDirname, "rotational_symmetry_segmentation");
  std::string resultDirname               = utl::fullfile(symmetrySegmentationDirname, outputDirname);
  std::string symmetryFilename            = utl::fullfile(resultDirname, "symmetries.txt");
  std::string segmentationFilename        = utl::fullfile(resultDirname, "segments.txt");
  
//  std::cout << sceneCloudFilename << std::endl;
//  std::cout << sceneDirname << std::endl;
//  std::cout << octomapFilename << std::endl;
//  std::cout << tablePlaneFilename << std::endl;
//  std::cout << resultDirname << std::endl;
//  std::cout << symmetryFilename << std::endl;
//  std::cout << segmentationFilename << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  ///////////////////////           PARAMETERS           ///////////////////////
  //////////////////////////////////////////////////////////////////////////////
  
  // Scene oversegmentation parameters
  utl::SmoothSegParams sceneOversegParams;
  sceneOversegParams.voxel_size = 0.005f;
  sceneOversegParams.min_segment_size = 120;
  sceneOversegParams.max_iou = 0.8f;
  sceneOversegParams.smoothness = { std::pair<float, float>(pcl::deg2rad(10.0f), 0.5f),
                                    std::pair<float, float>(pcl::deg2rad(15.0f), 0.5f) };

  // Reflectional symmetry detection parameters
  sym::ReflSymDetectParams reflDetParams;
  reflDetParams.voxel_size                  = 0.0f;
  reflDetParams.num_angle_divisions         = 5;
  reflDetParams.flatness_threshold          = 0.005f;
  reflDetParams.refine_iterations           = 20;
  
  reflDetParams.max_correspondence_reflected_distance = 0.01f;
  reflDetParams.max_occlusion_distance                = 0.03f;
  reflDetParams.min_inlier_normal_angle               = pcl::deg2rad(15.0f);
  reflDetParams.max_inlier_normal_angle               = pcl::deg2rad(20.0f);
    
  reflDetParams.max_occlusion_score           = 0.01f;
  reflDetParams.min_cloud_inlier_score        = 0.3f;
  reflDetParams.min_corresp_inlier_score      = 0.8f;
  
  reflDetParams.symmetry_min_angle_diff       = pcl::deg2rad(7.0);
  reflDetParams.symmetry_min_distance_diff    = 0.01f;
  reflDetParams.max_reference_point_distance  = 0.3f;
  
  // Reflectional symmetry segmentation parameters
  sym::ReflSymSegParams reflSegParams;
  reflSegParams.voxel_size = 0.0f;
  
  reflSegParams.max_sym_corresp_reflected_distance = 0.01f;
  reflSegParams.min_occlusion_distance = 0.01f;
  reflSegParams.max_occlusion_distance = 0.03f;
  reflSegParams.min_normal_fit_angle = pcl::deg2rad(10.0f);
  reflSegParams.max_normal_fit_angle = pcl::deg2rad(45.0f);
  
  reflSegParams.aw_radius = std::max(reflSegParams.voxel_size, 0.005f) * 2.0f;
  reflSegParams.aw_num_neighbors = 9;
  reflSegParams.aw_sigma_convex = 2.0f;
  reflSegParams.aw_sigma_concave = 0.15f;
  reflSegParams.fg_weight_importance  = 1.0f;
  reflSegParams.bg_weight_importance  = 2.0f;
  reflSegParams.bin_weight_importance = 5.0f;
  
  reflSegParams.max_symmetry_score    = 0.3f;
  reflSegParams.max_occlusion_score   = 0.005f;
  reflSegParams.max_smoothness_score  = 0.3f;
  reflSegParams.min_segment_size      = 200;
  reflSegParams.min_symmetry_support_overlap      = 0.5f;
  reflSegParams.similar_segment_iou_ = 0.95f;
  
  // Occupancy map parameters
  float occupancyMapBBXInflationRadius = 0.15f;                                    // Inflation radius of the distance map bounding box relative to the scene cloud bounding box  
  float occupancyMapMaxDistance = std::max(reflSegParams.max_occlusion_distance, reflDetParams.max_occlusion_distance);
  
  //////////////////////////////////////////////////////////////////////////////
  ///////////////////////           DATA LOAD            ///////////////////////
  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Loading data..." << std::endl;
  
  pcl::PointCloud<PointNC>::Ptr sceneCloudHighRes  (new pcl::PointCloud<PointNC>);
  if (pcl::io::loadPLYFile (sceneCloudFilename, *sceneCloudHighRes))
    return -1;
  
  // Read scene octree
  OccupancyMapPtr sceneOccupancyMap (new OccupancyMap);
  if (!sceneOccupancyMap->readOccupancyTree(octomapFilename))
    return -1;
  
  std::cout << "  Octomap resolution: " << sceneOccupancyMap->getOccupancyTreeResolution () << " metres" << std::endl;
  std::cout << "  Octomap depth: " << sceneOccupancyMap->getOccupancyTreeDepth() << std::endl;
  
  // Read tabple plane coefficients
  Eigen::Vector4f tablePlaneCoefficients;
  utl::readASCII(tablePlaneFilename, tablePlaneCoefficients);
  if (tablePlaneCoefficients.size () != 4)
  {
    std::cout << "Table plane coefficients must have 4 values, instead has " << tablePlaneCoefficients.size() << " values." << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Create a scene occupancy map
  //----------------------------------------------------------------------------
    
  std::cout << "Constructing scene occupancy map..." << std::endl;
  double start = pcl::getTime ();
  double totalStart = pcl::getTime ();

  // Get cloud bounding box size
  Eigen::Vector4f bbxMin, bbxMax;
  pcl::getMinMax3D(*sceneCloudHighRes, bbxMin, bbxMax);
  bbxMin.array() -= occupancyMapBBXInflationRadius;
  bbxMax.array() += occupancyMapBBXInflationRadius;
  
  // Initialize map
  sceneOccupancyMap->setBoundingPlanes(std::vector<Eigen::Vector4f> (1, tablePlaneCoefficients));
  sceneOccupancyMap->distanceMapFromOccupancy ( bbxMin.head(3), bbxMax.head(3),
                                                sceneOccupancyMap->getOccupancyTreeDepth(),
                                                occupancyMapMaxDistance,
                                                true  );
      
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
                                    
  //////////////////////////////////////////////////////////////////////////////
  ////////////////           SCENE OVERSEGMENTATION            /////////////////
  //////////////////////////////////////////////////////////////////////////////
  
  pcl::PointCloud<PointNC>::Ptr     sceneCloud                (new pcl::PointCloud<PointNC>);
  utl::Map                     downsampleMap;
  std::vector<utl::Map>        oversegSegments;
  utl::Map                     oversegSegmentsLinear;
  
  oversegmentScene<PointNC> (sceneCloudHighRes, sceneOversegParams, sceneCloud, downsampleMap, oversegSegments, oversegSegmentsLinear);

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////           REFLECTIONAL           //////////////////////
  //////////////////////////////////////////////////////////////////////////////
    
  //----------------------------------------------------------------------------
  // Symmetry detection
  //----------------------------------------------------------------------------
    
  std::cout << "Detecting reflectional symmetry..." << std::endl;
  start = pcl::getTime (); 
  
  std::vector<sym::ReflectionalSymmetry>  reflSymmetry;
  std::vector<std::vector<int> >          reflSymmetrySupport;
  
  if (  !detectReflectionalSymmetryScene<PointNC> ( sceneCloud,
                                                    sceneOccupancyMap,
                                                    oversegSegmentsLinear,
                                                    reflDetParams,
                                                    reflSymmetry,
                                                    reflSymmetrySupport ))
  {
    std::cout << "Could not detect reflectional symmetries." << std::endl;
    return -1;
  }

  std::cout << "  " << reflSymmetry.size() << " symmetries detected." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;  
  
  //----------------------------------------------------------------------------
  // Symmetry segmentation
  //----------------------------------------------------------------------------
      
  std::cout << "Segmenting reflectional objects..." << std::endl;
  start = pcl::getTime ();

  // Variables  
  utl::Map                   reflSegments;
  std::vector<int>                reflSegmentFilteredIds;
  std::vector<std::vector<int> >  reflSegmentMergedIds;
  
  sym::ReflectionalSymmetrySegmentation<PointNC> reflSeg;
  reflSeg.setInputCloud(sceneCloud);
  reflSeg.setInputOcuppancyMap(sceneOccupancyMap);
  reflSeg.setInputTablePlane(tablePlaneCoefficients);
  reflSeg.setInputSymmetries(reflSymmetry, reflSymmetrySupport);
  reflSeg.setParameters(reflSegParams);
  reflSeg.segment();
  reflSeg.getSegments(reflSegments, reflSegmentFilteredIds, reflSegmentMergedIds);

  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
  
  //----------------------------------------------------------------------------
  // Refine symmetries given segments
  //----------------------------------------------------------------------------

  std::cout << "Refining symmetry..." << std::endl;
  start = pcl::getTime ();
    
  std::vector<sym::ReflectionalSymmetry>  reflSymmetryRefined (reflSegments.size());
  
  # pragma omp parallel for
  for (size_t segId = 0; segId < reflSegments.size(); segId++)
  {
    if (reflSegments[segId].size() < 3)
    {
      reflSymmetryRefined[segId] = reflSymmetry[segId];
    }
    else
    {
      // Prepare input
      pcl::PointCloud<PointNC>::Ptr segmentCloud (new pcl::PointCloud<PointNC>);
      pcl::copyPointCloud<PointNC>(*sceneCloud, reflSegments[segId], *segmentCloud);
      std::vector<sym::ReflectionalSymmetry> symmetriesInitial (1, reflSymmetry[segId]);
      
      // Output variables
      std::vector<sym::ReflectionalSymmetry> symmetriesRefinedTMP;
      std::vector<int>  symmetryFilteredIdsTMP, symmetryMergedIdsTMP;
      
      // Refine
      sym::ReflectionalSymmetryDetection<PointNC> rsd (reflDetParams);
      rsd.setInputCloud(segmentCloud);
      rsd.setInputOcuppancyMap(sceneOccupancyMap);
      rsd.setInputSymmetries(symmetriesInitial);
      if (rsd.detect())
      {
        rsd.getSymmetries(symmetriesRefinedTMP, symmetryFilteredIdsTMP, symmetryMergedIdsTMP);
        reflSymmetryRefined[segId] = symmetriesRefinedTMP[0];
        pcl::PointCloud<PointNC>::Ptr segmentCloud (new pcl::PointCloud<PointNC>);
        pcl::copyPointCloud(*sceneCloud, reflSegments[segId], *segmentCloud);
        Eigen::Vector4f cloudMeanTMP;
        pcl::compute3DCentroid<PointNC>(*segmentCloud, cloudMeanTMP);
      }
      else
      {
        reflSymmetryRefined[segId] = reflSymmetry[segId];
      }
    }
  }
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;

  //----------------------------------------------------------------------------
  // Refine segments given refined symmetries
  //----------------------------------------------------------------------------

  std::cout << "Segmenting refined..." << std::endl;
  start = pcl::getTime ();
      
  std::vector<std::vector<int> >    reflSegmentsRefined;
  std::vector<int>                  reflSegmentFilteredIdsRefined;
  std::vector<std::vector<int> >    reflSegmentMergedIdsRefined;
  
  reflSeg.setInputSymmetries(reflSymmetryRefined, reflSymmetrySupport);
  reflSeg.segment();
  reflSeg.filter();
  reflSeg.merge();
  reflSeg.getSegments(reflSegmentsRefined, reflSegmentFilteredIdsRefined, reflSegmentMergedIdsRefined);
  
  // Get final segments
  utl::Map reflSegmentsFinal (reflSegmentMergedIdsRefined.size());
  std::vector<std::vector<sym::ReflectionalSymmetry> > reflSymmetryFinal (reflSegmentMergedIdsRefined.size());
  
  for (size_t segIdIt = 0; segIdIt < reflSegmentMergedIdsRefined.size(); segIdIt++)
  {
    reflSegmentsFinal[segIdIt] = reflSegmentsRefined[reflSegmentMergedIdsRefined[segIdIt][0]];
    
    for (size_t symIdIt = 0; symIdIt < reflSegmentMergedIdsRefined[segIdIt].size(); symIdIt++)
    {
      int symId = reflSegmentMergedIdsRefined[segIdIt][symIdIt];
      reflSymmetryFinal[segIdIt].push_back(reflSymmetryRefined[symId]);
    }
  }
  
  std::cout << "  " << reflSegmentMergedIdsRefined.size() << " merged refined segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;
  
  //----------------------------------------------------------------------------
  // Remove duplicate symmetries
  //----------------------------------------------------------------------------

  std::cout << "Remove duplicate symmetries..." << std::endl;
  start = pcl::getTime ();
  
  for (size_t segId = 0; segId < reflSymmetryFinal.size(); segId++)
  {
    std::vector<int> merged;
    
    // Get mean
    pcl::PointCloud<PointNC>::Ptr segmentCloud (new pcl::PointCloud<PointNC>);
    pcl::copyPointCloud<PointNC>(*sceneCloud, reflSegmentsFinal[segId], *segmentCloud);
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*segmentCloud, centroid);
    
    sym::mergeDuplicateReflSymmetries ( reflSymmetryFinal[segId],
                                        std::vector<Eigen::Vector3f> (reflSymmetryFinal[segId].size(), centroid.head(3)),
                                        std::vector<float> (reflSymmetryFinal[segId].size(), 1.0),
                                        merged,
                                        reflDetParams.symmetry_min_angle_diff,
                                        reflDetParams.symmetry_min_distance_diff,
                                        reflDetParams.max_reference_point_distance
                                      );
    
    std::vector<sym::ReflectionalSymmetry> symmetriesFiltered;
    for (size_t i = 0; i < merged.size(); i++)
      symmetriesFiltered.push_back(reflSymmetryFinal[segId][merged[i]]);
    
    reflSymmetryFinal[segId] = symmetriesFiltered;
  }
  
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  std::cout << "----------------------------" << std::endl;
  float execution_time = (pcl::getTime() - totalStart);
  std::cout << "Total time: " << execution_time << " seconds" << std::endl;

  // // Save timing information to a file.
  // std::ofstream outfile;
  // outfile.open("./reflectional_segmentation_timings.txt", std::ios_base::app);
  // outfile << utl::getBasename(sceneDirname) << ": " << execution_time << "\n";
  
  //////////////////////////////////////////////////////////////////////////////
  /////////////////////           VISUALIZATION           //////////////////////
  //////////////////////////////////////////////////////////////////////////////

  if (!visualize)
    return 0;
    
  //Print instructions
  std::cout << "-------------------------------" << std::endl;
  std::cout << "|   Visualization controls    |" << std::endl;
  std::cout << "-------------------------------" << std::endl;
  std::cout << "| NUMPAD keys 1-4   | switch between different processing steps" << std::endl;
  std::cout << "| Arrow keys        | switch between different segments/symmetries" << std::endl;
  std::cout << "| NUMPAD Delete     | visualize occlusion space" << std::endl;
  std::cout << "-------------------------------" << std::endl;

  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  visualizer.setCameraPosition (  0.0, 0.0, -1.0,   // camera position
                                  0.0, 0.0, 1.0,   // viewpoint
                                  0.0, -1.0, 0.0,   // normal
                                  0.0);            // viewport
  visualizer.setBackgroundColor (utl::bgColor.r, utl::bgColor.g, utl::bgColor.b);
  visualizer.registerKeyboardCallback(keyboard_callback, (void*)(&visState));  
  visState.updateDisplay_ = true;
    
  while (!visualizer.wasStopped())
  {
    // Update display if needed
    if (visState.updateDisplay_)
    {
      // First remove everything
      visualizer.removeAllPointClouds();
      visualizer.removeAllShapes();
      visualizer.removeAllCoordinateSystems();
      visState.updateDisplay_ = false;
            
      // Show occupancy
      if (visState.showOccupancy_)
      {
        sceneOccupancyMap->showObstacleSpaceBoundaryMesh(visualizer);
      }
      
      // Then add things as required
      if (visState.cloudDisplay_ == VisState::CLOUD)
      {
        utl::showPointCloudColor<PointNC>(visualizer, sceneCloud, "cloud", visState.pointSize_);
        if (visState.showNormals_)
          utl::showNormalCloud<PointNC>(visualizer, sceneCloud, 10, 0.02, "normals", visState.pointSize_, utl::green);
        
        visualizer.addText("Original cloud", 0, 150, 24, 1.0, 1.0, 1.0);
      }
      
      else if ( visState.cloudDisplay_ == VisState::INITIAL_OVERSEGMENTAION)
      {
        visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, oversegSegments.size()-1);
        int segParamId = visState.segIterator_;
        
        utl::showSegmentation<PointNC>(visualizer, sceneCloud, oversegSegments[segParamId], "segment", visState.pointSize_);

        visualizer.addText("Initial oversegmentation " + std::to_string(segParamId+1) + " / " + std::to_string(sceneOversegParams.smoothness.size()), 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText(std::to_string(oversegSegments[segParamId].size()) + " segments", 0, 125, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Normal variation: " + std::to_string(pcl::rad2deg(sceneOversegParams.smoothness[segParamId].first)), 0, 100, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Valid bin: " + std::to_string(sceneOversegParams.smoothness[segParamId].second), 0, 75, 24, 1.0, 1.0, 1.0);
      }
      
      else if ( visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES)
      {
        std::vector<std::vector<int> > symmetrySegmentsDisplay;
        std::vector<sym::ReflectionalSymmetry> symmetryDisplay;
        std::vector<int> symmetryDisplayIds;
        std::string text;

        visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, reflSymmetry.size()-1);
        int symId = visState.segIterator_;
        
        utl::showFGSegmentationColor<PointNC>(visualizer, sceneCloud, reflSymmetrySupport[symId], "object", visState.pointSize_);
        
        // Show symmetry
        if (visState.showSymmetry_)
          sym::showReflectionalSymmetry(visualizer, reflSymmetry[symId], "symmetry", 0.2);
        
        visualizer.addText("Reflectional symmetry ", 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Symmetry " + std::to_string(visState.segIterator_+1) + " / " + std::to_string(reflSymmetry.size()), 15, 125, 24, 1.0, 1.0, 1.0);        
      }
      
      else if ( visState.cloudDisplay_ == VisState::REFLECTIONAL_SEGMENTS)
      {
        visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, reflSegmentsFinal.size()-1);
        int segId = visState.segIterator_;
        
        utl::showFGSegmentationColor<PointNC>(visualizer, sceneCloud, reflSegmentsFinal[segId], "object", visState.pointSize_);

        if (visState.showSymmetry_)
        {
          for (size_t i = 0; i < reflSymmetryFinal[segId].size(); i++)
            sym::showReflectionalSymmetry(visualizer, reflSymmetryFinal[segId][i], "symmetry_"+std::to_string(i), 0.2);
        }
                
        visualizer.addText("Reflectional segments", 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Segment " + std::to_string(segId+1) + " / " + std::to_string(reflSegmentMergedIdsRefined.size()), 15, 125, 24, 1.0, 1.0, 1.0);  
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
  }
  
  return 0;
}