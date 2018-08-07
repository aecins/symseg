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
#include "symmetry/rotational_symmetry_segmentation.hpp"

#include "scene_oversegmentation.hpp"
#include "rotational_symmetry_detection_scene.hpp"

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
                                    
  // Rotatioanal symmetry detection parameters
  sym::RotSymDetectParams rotDetParams;
  rotDetParams.ref_max_fit_angle       = pcl::deg2rad(45.0f);
  rotDetParams.min_normal_fit_angle    = pcl::deg2rad(10.0f);
  rotDetParams.max_normal_fit_angle    = pcl::deg2rad(60.0f);
  rotDetParams.min_occlusion_distance  = 0.01f;
  rotDetParams.max_occlusion_distance  = 0.03f;
  rotDetParams.max_symmetry_score      = 0.02f;
  rotDetParams.max_occlusion_score     = 0.012f;
  rotDetParams.max_perpendicular_score = 0.6f;
  rotDetParams.min_coverage_score      = 0.3f;
  
  // Rotational segmentation parameters
  sym::RotSymSegParams rotSegParams;
  rotSegParams.voxel_size = 0.005f;
  rotSegParams.min_normal_fit_angle    = pcl::deg2rad(0.0f);    // Minimum symmetry error of fit for a point
  rotSegParams.max_normal_fit_angle    = pcl::deg2rad(15.0f);    // Minimum symmetry error of fit for a point
  rotSegParams.min_occlusion_distance = 0.005f;
  rotSegParams.max_occlusion_distance = 0.03f;
  rotSegParams.max_perpendicular_angle = pcl::deg2rad(30.0f);
  rotSegParams.aw_radius = rotSegParams.voxel_size * 2.0f;
  rotSegParams.aw_num_neighbors = 9;
  rotSegParams.aw_sigma_convex = 2.0f;
  rotSegParams.aw_sigma_concave = 0.15f;
  rotSegParams.fg_weight_importance  = 1.0f;
  rotSegParams.bg_weight_importance  = 1.0f;
  rotSegParams.bin_weight_importance = 2.0f;
  
  rotSegParams.max_symmetry_score    = 0.4f;
  rotSegParams.max_occlusion_score   = 0.03f;
  rotSegParams.max_smoothness_score  = 0.3f;
  rotSegParams.min_segment_size      = 100;
  
  // Occupancy map parameters
  float occupancyMapBBXInflationRadius = 0.15f;                                    // Inflation radius of the distance map bounding box relative to the scene cloud bounding box  
  float occupancyMapMaxDistance = std::max(rotSegParams.max_occlusion_distance, rotDetParams.max_occlusion_distance);
  
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
  ///////////////////////           ROTATIONAL           ///////////////////////
  //////////////////////////////////////////////////////////////////////////////
  
  //----------------------------------------------------------------------------
  // Symmetry detection
  //----------------------------------------------------------------------------
    
  std::cout << "Detecting rotational symmetry..." << std::endl;
  start = pcl::getTime (); 
  
  std::vector<sym::RotationalSymmetry>  rotSymmetry;
  std::vector<std::vector<int> >        rotSymmetrySupport;
  
  if (  !detectRotationalSymmetryScene<PointNC> ( sceneCloud,
                                                  sceneOccupancyMap,
                                                  oversegSegmentsLinear,
                                                  rotDetParams,
                                                  rotSymmetry,
                                                  rotSymmetrySupport ))
  {
    std::cout << "Could not detect rotational symmetries." << std::endl;
    return -1;
  }

  std::cout << "  " << rotSymmetry.size() << " symmetries detected." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;  
    
  //----------------------------------------------------------------------------
  // Symmetry segmentation
  //----------------------------------------------------------------------------
      
  std::cout << "Segmenting rotational objects..." << std::endl;
  start = pcl::getTime ();

  // Variables  
  utl::Map       rotSegments;
  std::vector<int>    rotSegmentFilteredIds;
  std::vector<float>  rotSymmetryScores, rotOcclusionScores, rotSmoothnessScores;
  
  sym::RotationalSymmetrySegmentation<PointNC> rotSeg;
  rotSeg.setInputCloud(sceneCloud);
  rotSeg.setInputOcuppancyMap(sceneOccupancyMap);
  rotSeg.setInputSymmetries(rotSymmetry);
  rotSeg.setParameters(rotSegParams);
  rotSeg.segment();
  rotSeg.filter();
  rotSeg.getSegments(rotSegments, rotSegmentFilteredIds);
  rotSeg.getScores(rotSymmetryScores, rotOcclusionScores, rotSmoothnessScores);

  std::cout << "  " << rotSegmentFilteredIds.size() << " filtered segments." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;  

  //////////////////////////////////////////////////////////////////////////////
  std::cout << "----------------------------" << std::endl;
  float execution_time = (pcl::getTime() - totalStart);
  std::cout << "Total time: " << execution_time << " seconds" << std::endl;

  // // Save timing information to a file.
  // std::ofstream outfile;
  // outfile.open("./rotational_segmentation_timings.txt", std::ios_base::app);
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

  // Visualize
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
        pcl::PointCloud<PointNC>::Ptr cloudDisplay (new pcl::PointCloud<PointNC>);
        std::string text;
        
        if (visState.cloudDisplay_ == VisState::CLOUD)
        {
          cloudDisplay = sceneCloud;
          text = "Original cloud";
        }

        utl::showPointCloudColor<PointNC>(visualizer, cloudDisplay, "cloud", visState.pointSize_);
        if (visState.showNormals_)
          utl::showNormalCloud<PointNC>(visualizer, cloudDisplay, 10, 0.02, "normals", visState.pointSize_, utl::green);
        
        visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
      }
      
      else if ( visState.cloudDisplay_ == VisState::INITIAL_OVERSEGMENTAION )
      {
        pcl::PointCloud<PointNC>::Ptr cloudDisplay (new pcl::PointCloud<PointNC>);
        std::vector<utl::Map> segmentationDisplay;
        std::string text;
        cloudDisplay = sceneCloud;
        segmentationDisplay = oversegSegments;
        text = "Initial oversegmentation ";

        visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, segmentationDisplay.size()-1);
        int segParamId = visState.segIterator_;
        
        utl::showSegmentation<PointNC>(visualizer, cloudDisplay, segmentationDisplay[segParamId], "segment", visState.pointSize_);

        visualizer.addText(text + std::to_string(segParamId+1) + " / " + std::to_string(sceneOversegParams.smoothness.size()), 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText(std::to_string(segmentationDisplay[segParamId].size()) + " segments", 0, 125, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Normal variation: " + std::to_string(pcl::rad2deg(sceneOversegParams.smoothness[segParamId].first)), 0, 100, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Valid bin: " + std::to_string(sceneOversegParams.smoothness[segParamId].second), 0, 75, 24, 1.0, 1.0, 1.0);
      }
      else if ( visState.cloudDisplay_ == VisState::ROTATIONAL_SYMMETRIES ||
                visState.cloudDisplay_ == VisState::ROTATIONAL_SEGMENTS )
      {
        std::vector<std::vector<int> > symmetrySegmentsDisplay;
        std::vector<sym::RotationalSymmetry> symmetryDisplay;
        std::vector<int> symmetryDisplayIds;
        std::string text;
        
        if (visState.cloudDisplay_ == VisState::ROTATIONAL_SYMMETRIES)
        {
          symmetryDisplay = rotSymmetry;
          symmetrySegmentsDisplay = rotSymmetrySupport;
          for (size_t symId = 0; symId < symmetryDisplay.size(); symId++)
            symmetryDisplayIds.push_back(symId);
          text = "Rotational symmetries";
        }
        else if (visState.cloudDisplay_ == VisState::ROTATIONAL_SEGMENTS)
        {
          symmetryDisplay = rotSymmetry;
          symmetrySegmentsDisplay = rotSegments;          
          symmetryDisplayIds = rotSegmentFilteredIds;
          text = "Rotational segments";
        }
        
        visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, symmetryDisplayIds.size()-1);
        int symId = symmetryDisplayIds[visState.segIterator_];
        
        utl::showFGSegmentationColor<PointNC>(visualizer, sceneCloud, symmetrySegmentsDisplay[symId], "object", visState.pointSize_);
                
        // Show symmetry
        if (visState.showSymmetry_)
          sym::showRotationalSymmetry(visualizer, symmetryDisplay[symId], "symmetry", 0.5, 5.0);
        
        visualizer.addText(text, 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Symmetry " + std::to_string(visState.segIterator_+1) + " / " + std::to_string(symmetryDisplayIds.size()), 15, 125, 24, 1.0, 1.0, 1.0);        
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
  }
  
  return 0;
}