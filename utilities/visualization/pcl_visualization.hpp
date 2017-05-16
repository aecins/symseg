// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef PCL_VISUALIZATION_UTILITIES_HPP
#define PCL_VISUALIZATION_UTILITIES_HPP

// PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkScalarBarActor.h>
#include <vtkActor2DCollection.h>
#include <vtkLine.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkRegularPolygonSource.h>
#include <vtkPlaneSource.h>

// Utilities
#include <std_vector.hpp>
#include <graph/graph.hpp>
#include <graph/graph_weighted.hpp>
#include <map.hpp>
#include <geometry/geometry.hpp>
#include <visualization/color.hpp>

namespace utl
{
  //----------------------------------------------------------------------------
  // Colors
  //----------------------------------------------------------------------------  

  // Frequently used colors
  const Color red   (1.0, 0.0, 0.0);
  const Color green (0.0, 1.0, 0.0);
  const Color blue  (0.0, 0.0, 1.0);
  const Color white (1.0, 1.0, 1.0);
  const Color black (0.0, 0.0, 0.0);
  const Color bgColor (0.7, 0.7, 1.0);    // Background color for the PCL visaualizer
  
//  /** \brief Convert a single point with RGB information to grayscale
//   *  \param[in,out]  point point to be converted to grayscale
//   *  \note conversion is done using the formula used in OpenCV (http://docs.opencv.org/modules/imgproc/doc/miscellaneous_transformations.html)
//   */
//  inline
//  std::vector<Color> colorizeData ( const std::vector<float> &data,
//                                    const int colormal_type = 0
//                                  )
//  {
//    // Get LUT
//    vtkSmartPointer<vtkLookupTable> lut;
//    pcl::visualization::getColormapLUT(static_cast<pcl::visualization::LookUpTableRepresentationProperties>(colormal_type), lut);
//
//    // Map data to colors
//    std::vector<Color> colors (data.size());
//    for (size_t i = 0; i < data.size(); i++)
//    {
//      double rgb[3];
//      lut->GetColor(static_cast<double>(data[i]), rgb);
//      colors[i] = Color(rgb[0], rgb[1], rgb[2]);
//    }
//
//    return colors;
//  }
  
  //----------------------------------------------------------------------------
  // Set rendering properties
  //----------------------------------------------------------------------------  
  
  /** \brief modify the rendering properties of a single normal pointcloud
   *  \param[in] visualizer   visualizer object
   *  \param[in] id           the point cloud object id prefix
   *  \param[in] point_size   size of the points used for visualization
   *  \param[in] color        color of the cloud
   *  \param[in] opacity      opacity of the cloud
   */
  inline
  void setPointCloudRenderProps  (  pcl::visualization::PCLVisualizer &visualizer,
                                    const std::string &id,
                                    const float point_size = -1.0,
                                    const Color &color = Color(),
                                    const float opacity = -1.0
                                  )
  {
    if (point_size > 0)
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
    if (color != Color())
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    if (opacity > 0)
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
  }

  /** \brief modify the rendering properties of a cloud of normals
   *  \param[in] visualizer   visualizer object
   *  \param[in] id           the point cloud object id prefix
   *  \param[in] line_width   width of normal arrows
   *  \param[in] color        color of the cloud
   *  \param[in] opacity      opacity of the cloud
   */
  inline
  void setNormalCloudRenderProps  ( pcl::visualization::PCLVisualizer &visualizer,
                                    const std::string &id,
                                    const float line_width = -1.0,
                                    const Color &color = Color(),
                                    const float opacity = -1.0
                                  )
  {
    if (line_width > 0)
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id);
    if (color != Color())
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    if (opacity > 0)
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
  }

  /** \brief modify the rendering properties of a line shape
   *  \param[in] visualizer   visualizer object
   *  \param[in] id           the point cloud object id prefix (default: cloud)
   *  \param[in] line_width   width of normal arrows
   *  \param[in] color        color of the cloud
   *  \param[in] opacity      opacity of the cloud
   *  \param[in] representation render representation (one of \ref{RenderingRepresentationProperties})
   */
  inline
  void setShapeRenderProps  ( pcl::visualization::PCLVisualizer &visualizer,
                              const std::string &id,
                              const Color &color = Color(),
                              const float opacity = -1.0,
                              const int representation = -1 
                          )
  {
    if (color != Color())
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    if (opacity > 0)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
    if (representation >= 0)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, representation, id);
  }    

  /** \brief modify the rendering properties of a line shape
   *  \param[in] visualizer   visualizer object
   *  \param[in] id           the point cloud object id prefix (default: cloud)
   *  \param[in] line_width   width of normal arrows
   *  \param[in] color        color of the cloud
   *  \param[in] opacity      opacity of the cloud
   */
  inline
  void setLineRenderProps ( pcl::visualization::PCLVisualizer &visualizer,
                            const std::string &id,
                            const float line_width = -1.0,
                            const Color &color = Color(),
                            const float opacity = -1.0
                          )
  {
    if (line_width > 0)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, line_width, id);
    if (color != Color())
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    if (opacity > 0)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);      
  }    
  
//  /** \brief modify the rendering properties of a colormap
//   *  \param[in] visualizer     visualizer object
//   *  \param[in] id             the point cloud object id prefix (default: cloud)
//   *  \param[in] colormal_type  colormap used \ref pcl::visualization::LookUpTableRepresentationProperties. If -1 colormap is unchanged
//   *  \param[in] range_auto     if true, colormap limits are set automatically from the data
//   *  \param[in] range_min      range minimum (if NaN colormap range is not updated)
//   *  \param[in] range_max      range maximum (if NaN colormap range is not updated)
//   *  \note this function sets colormap properties for clouds. It should be able to set colormap properties for shapes as well
//   */
//  inline
//  void setColormapRenderProps ( pcl::visualization::PCLVisualizer &visualizer,
//                                const std::string &id,
//                                const int colormal_type = -1,
//                                const bool  range_auto = false,
//                                const float range_min = std::numeric_limits<float>::quiet_NaN(),
//                                const float range_max = std::numeric_limits<float>::quiet_NaN()
//                              )
//  {
//    pcl::visualization::CloudActorMapPtr cloudActorMap = visualizer.getCloudActorMap();
//    pcl::visualization::ShapeActorMapPtr shapeActorMap = visualizer.getShapeActorMap();
//    if (cloudActorMap->find(id) != cloudActorMap->end())
//    {
//      if (colormal_type != -1)
//        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, colormal_type, id);
//      if (range_auto)
//        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO, id);
//      else if (!std::isnan(range_min) and !std::isnan(range_max))
//        visualizer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, range_min, range_max, id);
//    }
//    else if (shapeActorMap->find(id) != shapeActorMap->end())
//    {
//      if (colormal_type != -1)
//        visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, colormal_type, id);
//      if (range_auto)
//        visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, pcl::visualization::PCL_VISUALIZER_LUT_RANGE_AUTO, id);
//      // NOTE: this should be uncommented when proper support for shape LUT is implemented in PCL
////         else if (!std::isnan(range_min) and !std::isnan(range_max))
////           visualizer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT_RANGE, range_min, range_max, id);
//    }
//  }
  
  //----------------------------------------------------------------------------
  // Individual pointcloud visualization
  //----------------------------------------------------------------------------  

  /** \brief Show a single pointcloud
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  id          point cloud object id prefix (default: cloud)
   *  \param[in]  point_size  size of the points used for visualization
   *  \param[in]  color       color of the cloud
   *  \param[in]  opacity     opacity of the cloud
   */
  template <typename PointT>
  inline
  void showPointCloud ( pcl::visualization::PCLVisualizer &visualizer,
                        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                        const std::string &id = "cloud",
                        const float point_size = -1.0,
                        const Color &color = Color(),
                        const float opacity = -1.0
                      )
  {
    visualizer.addPointCloud<PointT>(cloud, id);
    setPointCloudRenderProps(visualizer, id, point_size, color, opacity);
  }

  /** \brief Show a single pointcloud with color information
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  id          point cloud object id prefix (default: cloud)
   *  \param[in]  point_size  size of the points used for visualization
   *  \param[in]  opacity     opacity of the cloud
   */
  template <typename PointT>
  inline
  void showPointCloudColor  ( pcl::visualization::PCLVisualizer &visualizer,
                              const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                              const std::string &id = "cloud",
                              const float point_size = -1.0,
                              const float opacity = -1.0
                            )
  {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> color_handler (cloud);
    visualizer.addPointCloud<PointT>(cloud, color_handler, id);
    setPointCloudRenderProps(visualizer, id, point_size, Color(), opacity);
  }

//  /** \brief visualize a pointcloud colored according to the scalar data vector
//    * \param[in]  visualizer    visualizer object
//    * \param[in]  cloud         pointcloud
//    * \param[in]  data          data vector
//    * \param[in]  id            the point cloud object id (default: cloud_seg)
//    * \param[in]  point_size    size of the point (default 1.0)
//    */
//  template <typename PointT, typename Scalar>
//  inline
//  void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
//                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
//                                const Eigen::Matrix<Scalar, 1, Eigen::Dynamic> &data,
//                                const std::string &id = "cloud",
//                                const float point_size = -1.0
//                              )
//  {
//    pcl::visualization::PointCloudColorHandlerCustomData<PointT, float> color_handler (cloud, data);
//    visualizer.addPointCloud<PointT> (cloud, color_handler, id);
//    setPointCloudRenderProps(visualizer, id, point_size);
//  }
  
//  /** \brief visualize a pointcloud colored according to the scalar data vector
//    * \param[in]  visualizer    visualizer object
//    * \param[in]  cloud         pointcloud
//    * \param[in]  data          data vector
//    * \param[in]  id            the point cloud object id (default: cloud_seg)
//    * \param[in]  point_size    size of the point (default 1.0)
//    */
//  template <typename PointT, typename Scalar>
//  inline
//  void showPointCloudWithData ( pcl::visualization::PCLVisualizer &visualizer,
//                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
//                                const std::vector<Scalar> &data,
//                                const std::string &id = "cloud",
//                                const float point_size = -1.0
//                              )
//  {
//    pcl::visualization::PointCloudColorHandlerCustomData<PointT, float> color_handler (cloud, data);
//    visualizer.addPointCloud<PointT> (cloud, color_handler, id);
//    setPointCloudRenderProps(visualizer, id, point_size);
//  }
  
  /** \brief Show pointcloud normals
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  level       display only every level'th point (default: 100)
   *  \param[in]  scale       the normal arrow scale (default: 0.02m)
   *  \param[in]  id          point cloud object id prefix (default: cloud)
   *  \param[in]  line_width  width of the normal lines
   *  \param[in]  color       color of the cloud
   *  \param[in]  opacity     opacity of the cloud
   */
  template <typename PointT>
  inline
  void showNormalCloud  ( pcl::visualization::PCLVisualizer &visualizer,
                          const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const int level = 100,
                          const float scale = 0.02f,
                          const std::string &id = "normals",
                          const float line_width = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                        )
  {
    visualizer.addPointCloudNormals<PointT>(cloud, level, scale, id);
    setNormalCloudRenderProps(visualizer, id, line_width, color, opacity);
  }

  /** \brief Show pointcloud normals for a specific subset of points in the
   * cloud.
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  indices     indices of points fow which normals will be displayed
   *  \param[in]  scale       the normal arrow scale (default: 0.02m)
   *  \param[in]  id          point cloud object id prefix (default: cloud)
   *  \param[in]  point_size  size of the points used for visualization
   *  \param[in]  color       color of the cloud
   *  \param[in]  opacity     opacity of the cloud
   */
  template <typename PointT>
  inline
  void showNormalCloud  ( pcl::visualization::PCLVisualizer &visualizer,
                          const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const std::vector<int> indices,
                          const float scale = 0.02f,
                          const std::string &id = "normals",
                          const float point_size = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                        )
  {
    typename pcl::PointCloud<PointT>::Ptr cloudIndexed (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud<PointT>(*cloud, indices, *cloudIndexed);
    showNormalCloud<PointT>(visualizer, cloudIndexed, 1, scale, id, point_size, color, opacity);
  }
  
  //----------------------------------------------------------------------------
  // Multiple pointcloud visualization
  //----------------------------------------------------------------------------  
  
  /** \brief visualize multiple pointclouds
   *  \param[in] visualizer  visualizer object
   *  \param[in] clouds      a vector of clouds that need to bi visualized
   *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
   *  \param[in] point_size  size of the points used for visualization
   *  \param[in] opacity     opacity of the displayed points
   */
  template <typename PointT>
  inline
  void showPointClouds (  pcl::visualization::PCLVisualizer &visualizer,
                          const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                          const std::string &id_prefix = "segment_",
                          const float point_size = -1.0,
                          const float opacity = -1.0
                        )
  {
    for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
    {
      Color color = getGlasbeyColor (cloudId);
      std::string id = id_prefix + "_" + std::to_string(cloudId);
      showPointCloud<PointT>(visualizer, clouds[cloudId], id, point_size, color, opacity);
    }
  }
  
  /** \brief visualize multiple pointclouds
   *  \param[in] visualizer  visualizer object
   *  \param[in] clouds      a vector of clouds that need to be visualized
   *  \param[in] color       color of the pointclouds
   *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
   *  \param[in] point_size  size of the points used for visualization
   *  \param[in] opacity     opacity of the displayed points
   */
  template <typename PointT>
  inline
  void showPointClouds (  pcl::visualization::PCLVisualizer &visualizer,
                          const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                          const Color &color,
                          const std::string &id_prefix = "segment_",
                          const float point_size = -1.0,
                          const float opacity = -1.0
                        )
  {
    for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
    {
      std::string id = id_prefix + "_" + std::to_string(cloudId);
      showPointCloud<PointT>(visualizer, clouds[cloudId], id, point_size, color, opacity);
    }
  }
  
  /** \brief visualize multiple pointclouds
   *  \param[in] visualizer  visualizer object
   *  \param[in] clouds      a vector of clouds that need to bi visualized
   *  \param[in] id_prefix   the point cloud object id prefix (default: segment_)
   *  \param[in] point_size  size of the points used for visualization
   *  \param[in] opacity     opacity of the displayed normals
   */
  template <typename PointT>
  inline
  void showNormalClouds ( pcl::visualization::PCLVisualizer &visualizer,
                          const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                          const int level = 100,
                          const float scale = 0.02f,
                          const std::string &id_prefix = "segment_",
                          const float point_size = -1.0,
                          const Color &color = Color(),
                          const float opacity = -1.0
                        )
  {
    for (size_t cloudId = 0; cloudId < clouds.size(); cloudId++)
    {
      std::string id = id_prefix + "_" + std::to_string(cloudId);
      showNormalCloud<PointT>(visualizer, clouds[cloudId], level, scale, id, point_size, color, opacity);
    }
  }    
  
  //----------------------------------------------------------------------------
  // Oversegmentation visualization
  //----------------------------------------------------------------------------  

  /** \brief Visualize multiple pointclouds that are a subset of some other cloud
   *  \param[in]  visualizer        visualizer object
   *  \param[in]  cloud             pointcloud
   *  \param[in]  segments          indices of points belonging to each segment
   *  \param[in]  id_prefix         the point cloud object id prefix (default: segment_)
   *  \param[in]  point_size        size of displayed points
   *  \param[in]  opacity           opacity of displayed points
   */
  template <typename PointT>
  inline
  void showSegmentation ( pcl::visualization::PCLVisualizer &visualizer,
                          const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const utl::Map &segments,
                          const std::string &id_prefix = "segment",
                          const float point_size = -1.0f,
                          const float bg_opacity = 0.1f
                        )
  {
    // Show foreground clouds
    std::vector<int> fgIndices;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> segmentClouds (segments.size());
    
    for (size_t segId = 0; segId < segments.size(); segId++)
    {
      fgIndices.insert(fgIndices.end(), segments[segId].begin(), segments[segId].end());
      
      // Copy cloud
      segmentClouds[segId].reset(new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, segments[segId], *segmentClouds[segId]);
    }
    showPointClouds<PointT>(visualizer, segmentClouds, id_prefix, point_size);      
    
    // Show background cloud
    std::vector<int> allIndices (cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      allIndices[pointId] = pointId;
    
    std::vector<int> bgIndices = utl::vectorDifference(allIndices, fgIndices);
    typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, bgIndices, *bgCloud);
    utl::showPointCloudColor<PointT>(visualizer, bgCloud, id_prefix + "_bg", point_size / 2.0f, bg_opacity);
  }  

  /** \brief Visualize multiple pointclouds that are a subset of some other colored cloud
   *  \param[in]  visualizer        visualizer object
   *  \param[in]  cloud             pointcloud
   *  \param[in]  segments          indices of points belonging to each segment
   *  \param[in]  id_prefix         the point cloud object id prefix (default: segment_)
   *  \param[in]  tint              tint of the color used to visualize each subcloud
   *  \param[in]  point_size        size of displayed points
   *  \param[in]  opacity           opacity of displayed points
   */
  template <typename PointT>
  inline
  void showSegmentationColored  ( pcl::visualization::PCLVisualizer &visualizer,
                                  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  const utl::Map &segments,
                                  const std::string &id_prefix = "segment",
                                  const float tint = 1.0,
                                  const float point_size = -1.0,
                                  const float bg_opacity = 0.1f
                                )
  {
    // Show foreground clouds
    std::vector<int> fgIndices;
    
    for (size_t segId = 0; segId < segments.size(); segId++)
    {
      fgIndices.insert(fgIndices.end(), segments[segId].begin(), segments[segId].end());
      
      // Copy cloud
      typename pcl::PointCloud<PointT>::Ptr curSegmentCloud (new pcl::PointCloud<PointT>);
      pcl::copyPointCloud(*cloud, segments[segId], *curSegmentCloud);
      
      // Color it
      utl::rgb2gray<PointT>(*curSegmentCloud);
      utl::tintPointCloud<PointT>(*curSegmentCloud, utl::getGlasbeyColor(segId), tint);
      
      // Display
      utl::showPointCloudColor<PointT>(visualizer, curSegmentCloud, id_prefix + "_" + std::to_string(segId), point_size);
    }
    
    // Show background cloud
    std::vector<int> allIndices (cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      allIndices[pointId] = pointId;
    
    std::vector<int> bgIndices = utl::vectorDifference(allIndices, fgIndices);
    typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, bgIndices, *bgCloud);
    utl::showPointCloudColor<PointT>(visualizer, bgCloud, id_prefix + "_bg", point_size / 2.0f, bg_opacity);
  }
  
  //----------------------------------------------------------------------------
  // Foreground segmentation visualization
  //----------------------------------------------------------------------------
  
  inline
  void showFGsegmentationMesh (pcl::visualization::PCLVisualizer &visualizer, const std::vector<pcl::PolygonMesh> &mesh, std::vector<int> &segmentation)
  { 
    // Get segmentation mask
    std::vector<bool> segmentationMask (mesh.size(), false);
    for (size_t segId = 0; segId < segmentation.size(); segId++)
      segmentationMask[segmentation[segId]] = true;
    
    Color FGcolor(1.0, 1.0, 1.0);
    Color BGcolor(0.2, 0.2, 0.2);
    
    for (size_t segId = 0; segId < mesh.size(); segId++)
    {
      std::string segIdStr = "segment_" + std::to_string(segId);
      visualizer.addPolygonMesh(mesh[segId], segIdStr);
      if (segmentationMask[segId])
        setPointCloudRenderProps(visualizer, segIdStr, -1.0, FGcolor);
      else
        setPointCloudRenderProps(visualizer, segIdStr, -1.0, BGcolor);
    }                  
  }  

  /** \brief Visualize a foreground background segmentation of a pointcloud.
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] fg_indices indices of the foreground points
   *  \param[in] id_prefix         the point cloud object id prefix (default: segment)* 
   *  \param[in] point_size size of the points used for visualization
   */  
  template <typename PointT>
  inline
  void showFGSegmentation ( pcl::visualization::PCLVisualizer &visualizer,
                            const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                            const std::vector<int> &fg_indices,
                            const std::string &id_prefix = "segment",
                            const int point_size = -1.0f,
                            const float bg_opacity = 0.1f
                          )
  {
    std::vector<bool> fg_mask (cloud->size(), false);
    for (size_t pointIt = 0; pointIt < fg_indices.size(); pointIt++)
      fg_mask[fg_indices[pointIt]] = true;
    
    // Show background cloud      
    std::vector<int> bg_indices;
    for (size_t pointId = 0; pointId < fg_mask.size(); pointId++)
      if (!fg_mask[pointId])
        bg_indices.push_back(pointId);
      
    typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);      
    pcl::copyPointCloud(*cloud, bg_indices, *bgCloud);
    utl::showPointCloud<PointT>(visualizer, cloud, id_prefix + "_bg", point_size / 2.0f, utl::black, bg_opacity);
          
    // Show foreground cloud
    typename pcl::PointCloud<PointT>::Ptr fgCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, fg_indices, *fgCloud);
    utl::showPointCloud<PointT>(visualizer, fgCloud, id_prefix + "_fg", point_size, utl::white);
  }
  
  /** \brief visualize a a freground background segmentation of a pointcloud where cloud is already colored
   *  \param[in] visualizer visualizer object
   *  \param[in] cloud input pointcloud
   *  \param[in] fg_indices indices of the foreground points
   *  \param[in] id_prefix         the point cloud object id prefix (default: segment)
   *  \param[in] point_size size of the points used for visualization
   *  \param[in] bg_color background color
   */
  template <typename PointT>
  inline
  void showFGSegmentationColor  ( pcl::visualization::PCLVisualizer &visualizer,
                                  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  const std::vector<int> &fg_indices,
                                  const std::string &id_prefix = "segment",
                                  const int point_size = -1.0f,
                                  const float bg_opacity = 0.1f
                                )
  {
    std::vector<bool> fg_mask (cloud->size(), false);
    for (size_t pointIt = 0; pointIt < fg_indices.size(); pointIt++)
      fg_mask[fg_indices[pointIt]] = true;
    
    // Show background cloud      
    std::vector<int> bg_indices;
    for (size_t pointId = 0; pointId < fg_mask.size(); pointId++)
      if (!fg_mask[pointId])
        bg_indices.push_back(pointId);
      
    typename pcl::PointCloud<PointT>::Ptr bgCloud (new pcl::PointCloud<PointT>);      
    pcl::copyPointCloud(*cloud, bg_indices, *bgCloud);
    
    for (size_t pointId = 0; pointId < bgCloud->size(); pointId++)
    {
      PointT curPoint = bgCloud->points[pointId];
      float gray = static_cast<uint8_t>(std::min(static_cast<float>(curPoint.r + curPoint.g + curPoint.b) / 3.0f, 255.0f));
      
      curPoint.r = gray;
      curPoint.g = gray;
      curPoint.b = gray;
      utl::tintPoint<PointT>(curPoint, utl::white, 0.3);
      
      bgCloud->points[pointId] = curPoint;
      

    }

    utl::showPointCloudColor<PointT>(visualizer, bgCloud, id_prefix + "_bg", point_size);
          
    // Show foreground cloud
    typename pcl::PointCloud<PointT>::Ptr fgCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, fg_indices, *fgCloud);
    utl::showPointCloudColor<PointT>(visualizer, fgCloud, id_prefix + "_fg", point_size);
  }
  
  //----------------------------------------------------------------------------
  // Graph visualization
  //----------------------------------------------------------------------------

  /** \brief visualize a graph defined on points in 3D space
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  points      points
   *  \param[in]  edges       edges in the graph
   *  \param[in]  id          id of the graph object (default: graph)
   *  \param[in]  line_width  width of the graph edge lines used for display (default 1.0)
   *  \param[in]  color       color of the graph edge lines
   *  \param[in]  opacity     opacity of the graph edge lines
   */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  void showPointGraph ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<PointT> &points,
                        const utl::GraphBase<NeighborT, EdgeT> &graph,
                        const std::string &id = "graph",
                        const float line_width = -1.0,
                        const Color &color = Color(0.3, 0.3, 0.3),
                        const float opacity = -1.0
                     )
  {
    // Create the polydata where we will store all the geometric data
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    
    // Create the points
    vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
    for (size_t pointId = 0; pointId < points.size(); pointId++)
    {
      double pt[3] = { points.points[pointId].x, points.points[pointId].y, points.points[pointId].z };
      points_vtk->InsertNextPoint(pt);
    }
    linesPolyData->SetPoints(points_vtk);   // Add the points to the polydata container
    
    // Create the lines
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
    {
      int vtx1Id, vtx2Id;
      if (!graph.getEdge(edgeId, vtx1Id, vtx2Id))
      {
        std::cout << "[utl::showPointGraph] could not visualize point graph." << std::endl;
        return;
      }        
      vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
      line->GetPointIds()->SetId(0, vtx1Id);
      line->GetPointIds()->SetId(1, vtx2Id);
      lines->InsertNextCell(line);
    }
    linesPolyData->SetLines(lines);         // Add the lines to the polydata container

    // Add polygon data to the visualizer
    visualizer.addModelFromPolyData (linesPolyData, id);
    utl::setLineRenderProps(visualizer, id, line_width, color, opacity);
  }
      
  /** \brief Visualize a weighted graph defined on points in 3D space
   *  \param[in]  visualizer    visualizer object
   *  \param[in]  points        points
   *  \param[in]  edges         graph edges
   *  \param[in]  edge_weights  edge weights
   *  \param[in]  id            id of the graph object (default: graph)
   *  \param[in]  line_width    width of the graph edge lines used for display (default 1.0)
   *  \param[in]  color         color of the graph edge lines
   *  \param[in]  opacity       opacity of the graph edge lines
   */  
  template <typename PointT>
  inline
  void showPointGraphWeighted ( pcl::visualization::PCLVisualizer &visualizer,
                                const pcl::PointCloud<PointT> &points,
                                const utl::GraphWeighted &graph,
                                const std::string &id = "graph",
                                const float line_width = -1.0,
                                const float opacity = -1.0
                              )
  {      
    // Create the polydata where we will store all the geometric data
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    
    // Create the points
    vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
    for (size_t pointId = 0; pointId < points.size(); pointId++)
    {
      double pt[3] = { points.points[pointId].x, points.points[pointId].y, points.points[pointId].z };
      points_vtk->InsertNextPoint(pt);
    }
    linesPolyData->SetPoints(points_vtk);   // Add the points to the polydata container
    
    // Create lines and scalars
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkFloatArray> colors = vtkSmartPointer<vtkFloatArray>::New ();
    for (size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
    {
      // Get edge properties
      int vtx1Id, vtx2Id;
      float weight;
      if (!graph.getEdge(edgeId, vtx1Id, vtx2Id, weight))
      {
        std::cout << "[utl::showPointGraph] could not visualize point graph." << std::endl;
        return;
      }
      
      // Add line
      vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
      line->GetPointIds()->SetId(0, vtx1Id);
      line->GetPointIds()->SetId(1, vtx2Id);
      lines->InsertNextCell(line);
      
      // Set line scalar
      colors->InsertTuple1 (edgeId, weight);
    }
    linesPolyData->SetLines(lines);         // Add the lines to the polydata container
    linesPolyData->GetCellData()->SetScalars(colors);   // Add scalars
    
    // Add polygon data to the visualizer
    visualizer.addModelFromPolyData (linesPolyData, id);
    utl::setLineRenderProps(visualizer, id, line_width, Color(), opacity);
  }
  
  /** \brief visualize a 3d curve represented as an ordered set of points
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  curve       curve
   *  \param[in]  id_prefix   prefix to be used for line objects (default: curve_)
   *  \param[in]  line_width  width of the lines used for display (default 1.0)
   *  \param[in]  color      color of the lines (default gray)
   */  
  template <typename PointT>
  inline
  void showCurve (pcl::visualization::PCLVisualizer &visualizer,
                  const pcl::PointCloud<PointT> &curve,
                  const std::string &id_prefix = "curve",
                  const float line_width = -1.0,
                  const Color &color = Color(0.4, 0.4, 0.4),
	    const float opacity = -1.0f
                )
  {
    if (curve.size() < 2)
    {
      std::cout << "[utl::showCurve] curve must contain at least two points!" << std::endl;
      return;
    }
    
    for (size_t linkId = 0; linkId < curve.size()-1; linkId++)
    {
      std::string id = id_prefix + "_" + std::to_string(linkId);
      visualizer.addLine(curve[linkId], curve[linkId+1], id);
      setLineRenderProps(visualizer, id, line_width, color, opacity);
    }
  }

  //----------------------------------------------------------------------------
  // Geometric primitives
  //----------------------------------------------------------------------------  

  /** \brief Add an oriented rectangle to the visualizer.
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  pose            rectangle pose (rectangle lies in the XY plane)
   *  \param[in]  width           width (along X axis)
   *  \param[in]  height          height (along Y axis)
   *  \param[in]  id              plane object id (default: plane)
   *  \param[in]  color           color of the displayed plane
   *  \param[in]  opacity         opacity of the displayed plane
   */    
  inline
  void showRectangle3d  ( pcl::visualization::PCLVisualizer &visualizer,
                          const Eigen::Affine3f &pose,
                          const float width   = 0.05,
                          const float height  = 0.05,
                          const std::string &id = "rectangle",
                          const Color color = Color(),
                          const float opacity = -1.0f
                        )
  {
    // First generate a square
    float halfWidth   = width / 2.0f;
    float halfHeight  = height / 2.0f;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_polygon (new pcl::PointCloud<pcl::PointXYZ>);
    plane_polygon->resize(4);
    plane_polygon->points[0] = pcl::PointXYZ(-halfWidth, -halfHeight,  0);
    plane_polygon->points[1] = pcl::PointXYZ( halfWidth, -halfHeight,  0);
    plane_polygon->points[2] = pcl::PointXYZ( halfWidth,  halfHeight,  0);
    plane_polygon->points[3] = pcl::PointXYZ(-halfWidth,  halfHeight,  0);

    // Transform polygon according to the pose
    pcl::transformPointCloud<pcl::PointXYZ>(*plane_polygon, *plane_polygon, pose);
    
    // Display
    visualizer.addPolygon<pcl::PointXYZ>(plane_polygon, id);
    setShapeRenderProps(visualizer, id, color, opacity);
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    if (color != Color())
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    
    if (opacity != -1.0f)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
    
    visualizer.addPolygon<pcl::PointXYZ>(plane_polygon, id + "_border");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id + "_border");
  }
  
  /** \brief Add an oriented rectangle to the visualizer.
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  point           point belonging to the plane
   *  \param[in]  normal          normal of the rectangle plane
   *  \param[in]  width           width (along X axis)
   *  \param[in]  height          height (along Y axis)
   *  \param[in]  id              plane object id (default: plane)
   *  \param[in]  color           color of the displayed plane
   *  \param[in]  opacity         opacity of the displayed plane
   */    
  inline
  void showRectangle3d  ( pcl::visualization::PCLVisualizer &visualizer,
                          const Eigen::Vector3f &point,
                          const Eigen::Vector3f &normal,
                          const float width   = 0.05,
                          const float height  = 0.05,
                          const std::string &id = "rectangle",
                          const Color color = Color(),
                          const float opacity = -1.0f
                        )
  {
    Eigen::Affine3f pose;
    pose.translation() = point;
    pose.linear() = utl::alignVectors<float>(Eigen::Vector3f::UnitZ(), normal);
    
    showRectangle3d(visualizer, pose, width, height, id, color, opacity);
  }    

  /** \brief Add a circle to the visualizer
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  center          circle center
   *  \param[in]  normal          normal of the circle plane
   *  \param[in]  radius          circle radius
   *  \param[in]  id              plane object id (default: plane)
   *  \param[in]  color           color of the displayed plane
   *  \param[in]  opacity         opacity of the displayed plane
   */    
  inline
  void showCircle3d ( pcl::visualization::PCLVisualizer &visualizer,
                      const Eigen::Vector3f &center,
                      const Eigen::Vector3f &normal,
                      const float radius,
                      const std::string &id = "circle",
                      const Color color = Color(),
                      const float opacity = -1.0f
                    )
  {
    // Create a circular polygon
    int numSteps = 50;
    float angularStep = M_PI * 2 / static_cast<float>(numSteps);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t stepId = 0; stepId < numSteps; stepId++)
    {
      float curAngle = angularStep * static_cast<float>(stepId);
      circle->push_back(pcl::PointXYZ (std::cos(curAngle) * radius, std::sin(curAngle) * radius, 0.0f));
    }

    // Transform it
    Eigen::Affine3f pose;
    pose.translation() = center;
    pose.linear() = utl::alignVectors<float>(Eigen::Vector3f::UnitZ(), normal);
    pcl::transformPointCloud<pcl::PointXYZ>(*circle, *circle, pose);
    
    // Display
    visualizer.addPolygon<pcl::PointXYZ>(circle, id);
    setShapeRenderProps(visualizer, id, color, opacity);
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, id);
    if (color != Color())
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, id);
    
    if (opacity != -1.0f)
      visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id);
  }

  //----------------------------------------------------------------------------
  // Miscelanious
  //----------------------------------------------------------------------------    
  
  /** \brief Show correspondences between points by rendering a line between
   * each pair of corresponding points.
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  source_points The source points
   *  \param[in]  target_points The target points
   *  \param[in]  correspondences The mapping from source points to target points. Each element must be an index into target_points
   *  \param[in]  nth display only the Nth correspondence (e.g., skip the rest)
   *  \param[in]  id the polygon object id (default: "correspondences")
   *  \param[in]  line_width   width of the lines
   *  \param[in]  opacity opacity of the lines
   */
  template <typename PointT> void
  showCorrespondences ( pcl::visualization::PCLVisualizer &visualizer,
                        const pcl::PointCloud<PointT> &source_points,
                        const pcl::PointCloud<PointT> &target_points,
                        const pcl::Correspondences &correspondences,
                        const int nth = 1,
                        const std::string &id = "correspondences",
                        const float line_width = -1.0f,
                        const float opacity = -1.0f
                      )
  {
    // Create the polydata where we will store all the geometric data
    vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();
    
    // Create the points
    vtkSmartPointer<vtkPoints> points_vtk = vtkSmartPointer<vtkPoints>::New();
    for (size_t crspId = 0; crspId < correspondences.size(); crspId++)
    {
      int srcId = correspondences[crspId].index_query;
      int tgtId = correspondences[crspId].index_match;
      double ptSrc[3] = { source_points.points[srcId].x, source_points.points[srcId].y, source_points.points[srcId].z };
      double ptTgt[3] = { target_points.points[tgtId].x, target_points.points[tgtId].y, target_points.points[tgtId].z };
      points_vtk->InsertNextPoint(ptSrc);
      points_vtk->InsertNextPoint(ptTgt);
    }
    linesPolyData->SetPoints(points_vtk);   // Add the points to the polydata container

    // Create the lines
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    for (size_t crspId = 0; crspId < correspondences.size(); crspId+=nth)
    {
      vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
      line->GetPointIds()->SetId(0, crspId*2);
      line->GetPointIds()->SetId(1, crspId*2+1);
      lines->InsertNextCell(line);
    }
    linesPolyData->SetLines(lines);         // Add the lines to the polydata container
    
    // Color the lines
    vtkSmartPointer<vtkUnsignedCharArray> colors =  vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    for (size_t crspId = 0; crspId < correspondences.size() / nth; crspId++)
    {
      utl::Color color = utl::getGlasbeyColor(crspId);
      unsigned char colorC[3];
      colorC[0] = static_cast<unsigned char>(color.r * 255.0f);
      colorC[1] = static_cast<unsigned char>(color.g * 255.0f);
      colorC[2] = static_cast<unsigned char>(color.b * 255.0f);
      colors->InsertNextTupleValue(colorC);
    }
    linesPolyData->GetCellData()->SetScalars(colors);      
    
    // Add polygon data to the visualizer
    visualizer.addModelFromPolyData (linesPolyData, id);
    utl::setLineRenderProps(visualizer, id, line_width, utl::Color(), opacity);      
  }
  
  /** \brief show a visualization of camera view frustum
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  K               camera matrix
   *  \param[in]  height          camera height
   *  \param[in]  width           camera width
   *  \param[in]  pose            camera pose
   *  \param[in]  id              name id of the camera
   */
  inline
  void showCamera ( pcl::visualization::PCLVisualizer &visualizer,
                    const Eigen::Matrix3f &K,
                    const int height, const int width,
                    const Eigen::Affine3f &pose,
                    const std::string &id = "cam",
                    const Color &color = Color ()
                  )
  {
    float focal = (K(0,0) + K(1,1)) / 2;
    float height_f = static_cast<float>(height);
    float width_f = static_cast<float>(width);
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x=0; p1.y=0; p1.z=0;
    float dist = 0.1;
    float minX, minY, maxX, maxY;
    maxX = dist*tan (std::atan (width_f / (2.0*focal)));
    minX = -maxX;
    maxY = dist*tan (std::atan (height_f / (2.0*focal)));
    minY = -maxY;
    p2.x=minX; p2.y=minY; p2.z=dist;
    p3.x=maxX; p3.y=minY; p3.z=dist;
    p4.x=maxX; p4.y=maxY; p4.z=dist;
    p5.x=minX; p5.y=maxY; p5.z=dist;
        
    p1=pcl::transformPoint (p1, pose);
    p2=pcl::transformPoint (p2, pose);
    p3=pcl::transformPoint (p3, pose);
    p4=pcl::transformPoint (p4, pose);
    p5=pcl::transformPoint (p5, pose);
    if (color == Color())
      visualizer.addText3D(id, p1, 0.01, 1.0, 1.0, 1.0, id);
    else
      visualizer.addText3D(id, p1, 0.01, color.r, color.g, color.b, id);

    visualizer.addLine (p1, p2, id + "_line1");
    visualizer.addLine (p1, p3, id + "_line2");
    visualizer.addLine (p1, p4, id + "_line3");
    visualizer.addLine (p1, p5, id + "_line4");
    visualizer.addLine (p2, p5, id + "_line5");
    visualizer.addLine (p5, p4, id + "_line6");
    visualizer.addLine (p4, p3, id + "_line7");
    visualizer.addLine (p3, p2, id + "_line8");
    
    setLineRenderProps(visualizer, id + "_line1", -1.0, color);
    setLineRenderProps(visualizer, id + "_line2", -1.0, color);
    setLineRenderProps(visualizer, id + "_line3", -1.0, color);
    setLineRenderProps(visualizer, id + "_line4", -1.0, color);
    setLineRenderProps(visualizer, id + "_line5", -1.0, color);
    setLineRenderProps(visualizer, id + "_line6", -1.0, color);
    setLineRenderProps(visualizer, id + "_line7", -1.0, color);
    setLineRenderProps(visualizer, id + "_line8", -1.0, color);
  }
  
  /** \brief show a visualization of camera view frustum
   *  \param[in]  visualizer      visualizer object
   *  \param[in]  K               camera matrix
   *  \param[in]  height          camera height
   *  \param[in]  width           camera width
   *  \param[in]  pose            camera pose
   *  \param[in]  id              name id of the camera
   */
  inline
  void removeCamera ( pcl::visualization::PCLVisualizer &visualizer, const std::string &id = "cam")
  {
    visualizer.removeText3D(id);
    visualizer.removeShape(id + "_line1");
    visualizer.removeShape(id + "_line2");
    visualizer.removeShape(id + "_line3");
    visualizer.removeShape(id + "_line4");
    visualizer.removeShape(id + "_line5");
    visualizer.removeShape(id + "_line6");
    visualizer.removeShape(id + "_line7");
    visualizer.removeShape(id + "_line8");
  }
  
  /** \brief Remove all text from the visualizer
   *  \param[in]  visualizer      visualizer object
   */    
  inline
  void removeAllText (pcl::visualization::PCLVisualizer &visualizer)
  {
    pcl::visualization::ShapeActorMapPtr shape_actor_map_ = visualizer.getShapeActorMap();
    pcl::visualization::ShapeActorMap::iterator am_it = shape_actor_map_->begin ();
    while (am_it != shape_actor_map_->end ())
    {
      if (am_it->second->IsA("vtkTextActor"))
      {
        if (visualizer.removeShape(am_it->first))
          am_it = shape_actor_map_->begin ();
        else
          std::cout << "[utl::removeAllText] could not remove a shave that was indetified as text. BAAAAaaaaAAaad!\n" << std::endl;
      }
      else
      {
        ++am_it;
      }
    }
  }
}
#endif // PCL_VISUALIZATION_UTILITIES_HPP
