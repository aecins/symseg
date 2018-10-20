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

// Symseg
#include "scene_oversegmentation.hpp"
#include "reflectional_symmetry_detection_scene.hpp"

// Proto
#include "google/protobuf/text_format.h"
#include "reflectional_symmetry_detection_example_options.pb.h"

// Project includes
#include "vis.hpp"

typedef pcl::PointXYZRGBNormal PointNC;

////////////////////////////////////////////////////////////////////////////////
void parseCommandLine(int argc, char** argv, std::string &inputCloudPath, std::string &outputDirnamePath, bool &visualize)
{
  inputCloudPath = "";
  outputDirnamePath = "";
  visualize = true;
  
  // Check parameters
  for (size_t i = 1; i < static_cast<size_t>(argc); i++)
  {
    std::string curParameter (argv[i]);
        
    if (curParameter == "-novis")
      visualize = false;   
    
        
    else if (inputCloudPath == "" && curParameter[0] != '-')
      inputCloudPath = curParameter;

    else if (outputDirnamePath == "" && curParameter[0] != '-')
      outputDirnamePath = curParameter;
    
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
  
  std::string inputCloudPath, outputDirnamePath;
  bool visualize;
  parseCommandLine(argc, argv, inputCloudPath, outputDirnamePath, visualize);

  //----------------------------------------------------------------------------
  // Generate paths and check command line arguments
  //----------------------------------------------------------------------------
    
  if (!utl::isFile(inputCloudPath))
  {
    std::cout << "Couldn't find pointcloud file '" << inputCloudPath << "'" << std::endl;
    return -1;
  }
  
  //----------------------------------------------------------------------------
  // Parameters
  //----------------------------------------------------------------------------
  
  // NOTE: you can change the parameters of the algorithm by modifying the
  // values in the file below.
  std::string options_file_path = "../examples/reflectional_detection/config.pbtxt";
  if (!utl::isFile(options_file_path))
  {
    std::cout << "Could not find config file " << options_file_path << ".";
    return -1;
  }

  proto::ReflectionalSymmetryDetectionExampleOptions options;
  std::ifstream input(options_file_path);
  std::stringstream input_string;
  input_string << input.rdbuf();  
  google::protobuf::TextFormat::ParseFromString(input_string.str(), &options);
  
  // Reflectional symmetry detection parameters
  sym::ReflSymDetectParams reflDetParams(options.reflectional_symmetry_detection_options());
  reflDetParams.voxel_size                  = options.voxel_size() * 2;   // Voxel size used in voxelgrid downsample.  
  reflDetParams.max_correspondence_reflected_distance =                   // Maximum allowed distance between the reflections of two points forming a symmetric 
                                        reflDetParams.voxel_size / 2;
    
  //----------------------------------------------------------------------------
  // Load data.
  //----------------------------------------------------------------------------
  
  std::cout << "Loading data..." << std::endl;
  
  pcl::PointCloud<PointNC>::Ptr sceneCloudHighRes  (new pcl::PointCloud<PointNC>);
  if (pcl::io::loadPLYFile (inputCloudPath, *sceneCloudHighRes))
    return -1;

  //----------------------------------------------------------------------------
  // Demean and rescale the scene.
  //----------------------------------------------------------------------------
  
  pcl::PCA<PointNC> pca;
  pca.setInputCloud(sceneCloudHighRes);

  // Find pointcloud diagonal length.
  pcl::PointCloud<PointNC>::Ptr sceneCloudHighResAligned  (new pcl::PointCloud<PointNC>);
  pca.project(*sceneCloudHighRes, *sceneCloudHighResAligned);
  
  PointNC bbxMin, bbxMax;
  pcl::getMinMax3D(*sceneCloudHighResAligned, bbxMin, bbxMax);
  float diagonalLength = (bbxMax.getVector3fMap() - bbxMin.getVector3fMap()).norm();
  float scalingFactor = options.downsample_diagonal_length() / diagonalLength;

  // Find pointcloud mean.
  Eigen::Vector4f cloudCentroid = pca.getMean();
  
  // Demean and scal pointcloud
  for (size_t point_id = 0; point_id < sceneCloudHighRes->size(); point_id++)
  {
    sceneCloudHighRes->points[point_id].getVector3fMap() -= cloudCentroid.head(3);
    sceneCloudHighRes->points[point_id].getVector3fMap() *= scalingFactor;
  }
    
  //----------------------------------------------------------------------------
  // Downsample the scene.
  //----------------------------------------------------------------------------
  
  double start = pcl::getTime ();
  double totalStart = pcl::getTime ();
  
  pcl::PointCloud<PointNC>::Ptr     sceneCloud                (new pcl::PointCloud<PointNC>);  
  
  std::cout << "Downsampling input pointcloud..." << std::endl;
  utl::Downsample<PointNC> ds;
  ds.setInputCloud(sceneCloudHighRes);
  ds.setDownsampleMethod(utl::Downsample<PointNC>::AVERAGE);
  ds.setLeafSize(options.voxel_size());
  ds.filter(*sceneCloud);
  std::cout << sceneCloudHighRes->size() << " points in original cloud." << std::endl;
  std::cout << sceneCloud->size() << " points after downsampling." << std::endl;
      
  //----------------------------------------------------------------------------
  // Reflectional symmetry detection
  //----------------------------------------------------------------------------
    
  std::cout << "Detecting reflectional symmetry..." << std::endl;
  start = pcl::getTime (); 
  
  std::vector<sym::ReflectionalSymmetry>  reflSymmetry;
  std::vector<std::vector<int> >          reflSymmetrySupport;
  utl::Map dummy_segments (1);
  for (size_t point_id = 0; point_id < sceneCloud->size(); point_id++) {
    dummy_segments[0].push_back(point_id);
  }
  
  if (  !detectReflectionalSymmetryScene<PointNC> ( sceneCloud,
                                                    nullptr,
                                                    dummy_segments,
                                                    reflDetParams,
                                                    reflSymmetry,
                                                    reflSymmetrySupport ))
  {
    std::cout << "Could not detect reflectional symmetries." << std::endl;
    return -1;
  }

  std::cout << "  " << reflSymmetry.size() << " symmetries detected." << std::endl;
  std::cout << "  " << (pcl::getTime() - start) << " seconds." << std::endl;  

  std::cout << "----------------------------" << std::endl;
  float execution_time = (pcl::getTime() - totalStart);
  std::cout << "Total time: " << execution_time << " seconds" << std::endl;

  //----------------------------------------------------------------------------
  // Save results to file
  //----------------------------------------------------------------------------

  // Before saving the symmetries we need to undo the effects of scaling and 
  // demeaning the scene.
  std::vector<sym::ReflectionalSymmetry>  reflSymmetryUnprojected;
  
  for (size_t sym_id = 0; sym_id < reflSymmetry.size(); sym_id++)
  {
    sym::ReflectionalSymmetry symmetryUnprojected = reflSymmetry[sym_id];
    Eigen::Vector3f origin = symmetryUnprojected.getOrigin();
    symmetryUnprojected.setOrigin(origin / scalingFactor + cloudCentroid.head(3));
    reflSymmetryUnprojected.push_back(symmetryUnprojected);
  }
  
  // Save result to file.
  if (!outputDirnamePath.empty()) {
    std::string outputFilePath = utl::fullfile(outputDirnamePath, "symmetries.txt");
    std::cout << "Saving segmentation results to " << outputFilePath << std::endl;
    sym::writeSymmetriesToFile(reflSymmetryUnprojected, outputFilePath);
  }
  
  // // Save timing information to a file.
  // std::ofstream outfile;
  // outfile.open("./reflectional_segmentation_timings.txt", std::ios_base::app);
  // outfile << utl::getBasename(sceneDirname) << ": " << execution_time << "\n";
  
  //----------------------------------------------------------------------------
  // Visualization.
  //----------------------------------------------------------------------------

  if (!visualize)
    return 0;
    
  //Print instructions
  std::cout << "-------------------------------" << std::endl;
  std::cout << "|   Visualization controls    |" << std::endl;
  std::cout << "-------------------------------" << std::endl;
  std::cout << "| NUMPAD keys 1-2   | switch between different processing steps" << std::endl;
  std::cout << "| Arrow keys        | switch between different segments/symmetries" << std::endl;
  std::cout << "| NUMPAD Delete     | visualize occlusion space" << std::endl;
  std::cout << "-------------------------------" << std::endl;

  VisState visState;
  pcl::visualization::PCLVisualizer visualizer;
  Eigen::Vector4f cloudCentroidVis;
  pcl::compute3DCentroid<PointNC>(*sceneCloud, cloudCentroidVis);
  
  visualizer.setCameraPosition (  0.0, 0.0, -1.0,   // camera position
                                  cloudCentroidVis[0], cloudCentroidVis[1], cloudCentroidVis[2],   // viewpoint
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
      
      // Then add things as required
      if (visState.cloudDisplay_ == VisState::CLOUD)
      {
        utl::showPointCloudColor<PointNC>(visualizer, sceneCloud, "cloud", visState.pointSize_);
        if (visState.showNormals_)
          utl::showNormalCloud<PointNC>(visualizer, sceneCloud, 10, options.voxel_size(), "normals", visState.pointSize_, utl::green);
        
        visualizer.addText("Original cloud", 0, 150, 24, 1.0, 1.0, 1.0);
      }

      else if ( visState.cloudDisplay_ == VisState::REFLECTIONAL_SYMMETRIES)
      {
        if (reflSymmetry.size() > 0) {
          visState.segIterator_ = utl::clampValueCircular<int>(visState.segIterator_, 0, reflSymmetry.size()-1);
          int symId = visState.segIterator_;          
          utl::showPointCloudColor<PointNC> (visualizer, sceneCloud, "object", visState.pointSize_);

          // Show symmetry
          if (visState.showSymmetry_)
            sym::showCloudReflectionalSymmetry<PointNC>(visualizer, sceneCloud, reflSymmetry[symId], "symmetry", 1.0);
          
          if (visState.showReconstructedCloud_) {
            pcl::PointCloud<PointNC>::Ptr sceneCloudReflected  (new pcl::PointCloud<PointNC>);
            reflSymmetry[symId].reflectCloud(*sceneCloud, *sceneCloudReflected);
            utl::showPointCloud<PointNC>(visualizer, sceneCloudReflected, "cloud_reflected", visState.pointSize_, utl::blue);
          }
          
        } else {
          visState.segIterator_ = -1;
          utl::showPointCloudColor<PointNC>(visualizer, sceneCloud, "cloud", visState.pointSize_);
          if (visState.showNormals_)
            utl::showNormalCloud<PointNC>(visualizer, sceneCloud, 10, 0.02, "normals", visState.pointSize_, utl::green);
        }
                            
        visualizer.addText("Reflectional symmetry ", 0, 150, 24, 1.0, 1.0, 1.0);
        visualizer.addText("Symmetry " + std::to_string(visState.segIterator_+1) + " / " + std::to_string(reflSymmetry.size()), 15, 125, 24, 1.0, 1.0, 1.0);        
      }
    }
    
    // Spin once
    visualizer.spinOnce();
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
  }
  
  return 0;
}