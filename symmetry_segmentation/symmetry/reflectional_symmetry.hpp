// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REFLECTIONAL_SYMMETRY_HPP
#define REFLECTIONAL_SYMMETRY_HPP

// STD includes
#include <math.h>
#include <fstream>

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/common/pca.h>

// utilities
#include <filesystem/filesystem.hpp>
#include <pointcloud/pointcloud.hpp>
#include <visualization/pcl_visualization.hpp>


namespace sym
{  
  /** \brief Class representing a reflectional symmetry in 3D space. A symmetry
   * is represented as a 3D plane.
   */  
  class ReflectionalSymmetry
  {
  public:
    
    /** \brief An empty constructor */
    ReflectionalSymmetry ()
      : origin_ (Eigen::Vector3f::Zero())
      , normal_ (Eigen::Vector3f::Zero())
    {};
    
    /** \brief A constructor from origin point and normal
     *  \param[in] origin origin point
     *  \param[in] normal symmetry plane normal
     */    
    ReflectionalSymmetry (const Eigen::Vector3f &origin, const Eigen::Vector3f &normal)
      : origin_ (origin)
      , normal_ (normal.normalized())
    { };

    /** \brief A constructor from plane coefficients (ax + by + cz = d)
     *  \param[in] plane_coefficients plane coefficients
     */
    ReflectionalSymmetry (const Eigen::Vector4f &plane_coefficients)
    {
      utl::planeCoefficientsToPointNormal<float>(plane_coefficients, origin_, normal_);
    };    
        
    /** \brief Get the normal vector describing the symmetry
     *  \return normal of the symmetry plane
     */    
    Eigen::Vector3f getNormal () const { return normal_; }
    
    /** \brief Get the origin point of the symmetry
     *  \return symmetry origin point
     */        
    Eigen::Vector3f getOrigin () const { return origin_; }
    
    /** \brief Set the origin point of the symmetry
     *  \param[in] origin symmetry origin point
     */
    void setOrigin   (const Eigen::Vector3f origin )
    {
      origin_  = origin;
    }
    
    /** \brief Set the normal point of the symmetry
     *  \param[in] normal symmetry normal
     */    
    void setNormal  (const Eigen::Vector3f normal)
    {
      normal_ = normal;
    }
    
    /** \brief Project a 3D point on the symmetry plane
     *  \param[in] point  point to be projected
     *  \return projected point
     */    
    Eigen::Vector3f projectPoint  (const Eigen::Vector3f &point)  const
    {
      return utl::projectPointToPlane<float>(point, origin_, normal_);
    };
    
    /** \brief Set the origin of the symmetry to be the input point projected on the current symmetry axis
     *  \param[in] point  point who's projection will be used as new origin
     */
    void setOriginProjected  (const Eigen::Vector3f &point)
    {
      origin_ = projectPoint(point);
    };
    
    /** \brief Generate a symmetry from two points
     *  \param[in] point1f first point
     *  \param[in] point2 second point
     */
    inline  
    void fromTwoPoints  ( const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
    {  
      Eigen::Vector3f origin = (point1 + point2) / 2;
      Eigen::Vector3f normal = point1 - point2;
      normal.normalize();
      *this = ReflectionalSymmetry(origin, normal);
    }

    /** \brief Transform the symmetry axis with an 3D rigid transformation.
     *  \param[in]  transform 3D affine transform
     */
    inline
    ReflectionalSymmetry transform  ( const Eigen::Affine3f &transform  ) const
    {
      return ReflectionalSymmetry(transform * origin_, transform.rotation() * normal_);
    }    

    /** \brief Write symmetry parameters to a filestream in ASCII format.
     *  \param[in] filestream filestream
     */
    void writeASCII (std::ofstream &filestream) const
    {
      filestream << "point:\n";
      filestream << "   " << origin_[0] << "  " << origin_[1] << "  " << origin_[2] << "  " << std::endl;
      filestream << "normal:\n";
      filestream << "   " << normal_[0] << "  " << normal_[1] << "  " << normal_[2] << "  " << std::endl;
    }
    
    /** \brief Write symmetry parameters to a file in ASCII format.
     *  \param[in] file filename
     */
    bool writeASCII (const std::string filename) const
    {
      // Check that parent directory exists
      std::string parentDirname = utl::getParentDir(filename);
      if (!utl::exists(parentDirname))
      {
        std::cout << "[sym::ReflectionalSymmetry::writeASCII] Parent directory does not exist ('" << parentDirname << "')." << std::endl;
        return false;
      }
      
      // Write to file
      std::ofstream file(filename);
      if (!file.is_open())
      {
        std::cout << "[sym::ReflectionalSymmetry::writeASCII] Could not open file for writing ('" << filename << "')." << std::endl;
        return false;
      }
      else
      {
        writeASCII(file);
      }
      file.close();
      
      return true;
    }
    
    /** \brief Read symmetry parameters from an ASCII file. Filestream pointer
     * must point to a line starting a valid symmetry record.
     *  \param[in] file input filestream
     */    bool readASCII (std::ifstream &filestream)
    {
      std::string line;
      std::istringstream iss;
            
      // Read point
      std::getline(filestream,line);
      if (line != "point:")
      {
        std::cout << "[sym::ReflectionalSymmetry::readASCII] 'point:' line does not match the expected format (" << line << ")." << std::endl;
        return false;
      }
      std::getline(filestream,line);
      iss.str(line);
      iss >> origin_[0] >> origin_[1] >> origin_[2];

      // Read direction
      std::getline(filestream,line);
      if (line != "normal:")
      {
        std::cout << "[sym::ReflectionalSymmetry::readASCII] 'normal:' line does not match the expected format  (" << line << ")." << std::endl;
        return false;
      }
      std::getline(filestream,line);
      iss.str(line);
      iss >> normal_[0] >> normal_[1] >> normal_[2];
      
      return true;
    }
    
    /** \brief Read symmetry parameters from an ASCII file.
     *  \param[in] file input filestream
     */
    bool readASCII (const std::string &filename)
    {
      bool readSuccess;
      
      // Check that file exists
      std::ifstream file(filename);
      if (!file.is_open())
      {
        std::cout << "[sym::ReflectionalSymmetry::readASCII] Could not open file for reading ('" << filename << "')." << std::endl;
        return false;
      }
      else
      {
        readSuccess = readASCII(file);
      }
      file.close();
      
      return readSuccess;
    }
    
    /** \brief Return the coefficients of equation of symmetry plane in the form
     * of ax + by + cz = d
     *  \return coefficients of a plane
     */
    Eigen::Vector4f getPlaneCoefficients () const 
    {
      Eigen::Vector4f plane;
      plane.head(3) = normal_;
      plane[3] = -origin_.dot(normal_);
      return plane;
    }
    
    /** \brief Get the signed distance between a symmetry plane and a point. The
     * sign of the distance depends on the direction of the symetry plane normal.
     *  \param[in] point  point
     *  \return signed distance
     */    
    float pointSignedDistance (const Eigen::Vector3f &point)  const
    {
      return utl::pointToPlaneSignedDistance<float>(point, origin_, normal_);
    };

    /** \brief Calculate the difference between two reflectional symmetries. Two
     * measures are calculated:
     * 1) Angle between symmetry planes
     * 2) Distance between the projections of a reference point onto the two
     *    symmetry planes
     *  \param[in]  symmetry_other second symmetry
     *  \param[in]  reference_point reference point
     *  \param[out]  angle     angle between symmetry planes
     *  \param[out]  distance  distance between projections of the reference point
     *  \note maximum angle between two symmetry axes is 90 degrees
     */
    inline
    void reflSymDifference (const ReflectionalSymmetry &symmetry_other, const Eigen::Vector3f &reference_point, float &angle, float &distance)
    {
      // Get angle between normals
      angle = utl::lineLineAngle<float>(normal_, symmetry_other.getNormal());      
      Eigen::Vector3f refPointProj1 = projectPoint(reference_point);
      Eigen::Vector3f refPointProj2 = symmetry_other.projectPoint(reference_point);
      distance = utl::pointToPointDistance<float>(refPointProj1, refPointProj2);
    }
    
    /** \brief Reflect a point around a symmetry plane
     *  \param[in] point original cloud
     *  \return reflected point
     */  
    inline
    Eigen::Vector3f reflectPoint (const Eigen::Vector3f &point) const
    {
      return (point - 2 * normal_ * (normal_.dot(point - origin_)));    
    }
    
    /** \brief Reflect a normal around a symmetry plane
     *  \param[in] normal original cloud
     *  \return reflected normal
     */  
    inline
    Eigen::Vector3f reflectNormal (const Eigen::Vector3f &normal) const
    {
      return (normal - 2 * (normal.dot(normal_) * normal_));
    }
    
    /** \brief Reflect a given pointcloud around a symmetry plane
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out reflected cloud
     */
    template <typename PointT>
    inline
    void reflectCloud(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out)  const
    {
      cloud_out = cloud_in;
      for (size_t pointId = 0; pointId < cloud_out.size(); pointId++)
        cloud_out.points[pointId].getVector3fMap() = reflectPoint(cloud_out.points[pointId].getVector3fMap());
    }

    /** \brief Reflect a given pointcloud around a symmetry plane
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out reflected cloud
     */
    template <typename PointT>
    inline
    void reflectCloudWithNormals(const typename pcl::PointCloud<PointT> &cloud_in, typename pcl::PointCloud<PointT> &cloud_out) const
    {
      cloud_out = cloud_in;
      for (size_t pointId = 0; pointId < cloud_in.size(); pointId++)
      {
        cloud_out.points[pointId].getVector3fMap()        = reflectPoint  (cloud_out.points[pointId].getVector3fMap());
        cloud_out.points[pointId].getNormalVector3fMap()  = reflectNormal (cloud_out.points[pointId].getNormalVector3fMap());
      }
    }

    /** \brief Calculate angle between normal 1 and normal 2 reflected by a symmetry
     *  \param[in] normal1 first normal
     *  \param[in] normal2 second normal
     *  \param[in] consistent_normals flag indicating if normals are consistently oriented or not (default true)
     *  \return angle (in radians) between two normals given the symmetry
     *  \note normals are assumed to be uniquely oriented (i.e. n and -n are different normals)
     */
//     template <typename PointT>    // NOTE: this is only required so that there is no conflict with the next method
//     inline  
//     float reflectedNormalAngle  ( const Eigen::Vector3f &normal1,
//                                   const Eigen::Vector3f &normal2,
//                                   const bool &consisntent_normals = true
//                                 ) const
//     { 
//       Eigen::Vector3f normal2reflected = this->reflectNormal(normal2);
//       float dotProd = utl::clampValue(normal1.dot(normal2reflected), -1.0f, 1.0f);
//       if (consisntent_normals)
//         return std::acos(dotProd);
//       else
//         return std::acos(std::abs(dotProd));
//     }

    /** \brief Calculate angle between normal 1 and normal 2 reflected by a symmetry
     *  \param[in] point1 point containing first normal
     *  \param[in] point2 point containing second normal
     *  \param[in] consistent_normals flag indicating if normals are consistently oriented or not (default true)
     *  \return angle (in radians) between two normals given the symmetry
     */  
//     template <typename PointT>
//     inline  
//     float reflectedNormalAngle  ( const PointT &point1,
//                                   const PointT &point2,
//                                   const bool &consisntent_normals = true
//                                 ) const
//     {    
//       return this->reflectedNormalAngle<PointT> ( point1.getNormalVector3fMap(), point2.getNormalVector3fMap(), consisntent_normals);
//     }    
        
  protected:
        
    // Member variables
    Eigen::Vector3f origin_;  ///< Point belonging to the symmetry plane. Symmetry is visualized around this point.
    Eigen::Vector3f normal_;  ///< Normal of the symmetry plane. Note that normal is always reoriented such that it's x coordinate is non-negative
  };
  
  /** \brief Print symmetry details to ostream */
  std::ostream& operator<< ( std::ostream& os, const ReflectionalSymmetry& symmetry)
  {
    os << "origin:             " << symmetry.getOrigin().transpose();
    os << std::endl;
    os << "normal:             " << symmetry.getNormal().transpose();
      
    return os;
  }
  
  /** \brief Visualize symmetry as a rectangular polygon
   *  \param[in] visualizer object
   *  \param[in] symmetry reflectional symmetry
   *  \param[in] id symmetry plane object id (default: symmetry)
   *  \param[in] side_width side length of the symmetry plane square
   */
  inline
  void showReflectionalSymmetry ( pcl::visualization::PCLVisualizer &visualizer,
                                  const ReflectionalSymmetry &symmetry,
                                  const std::string id = "symmetry",
                                  const float side_width = 0.05,
                                  utl::Color color = utl::Color(0.0, 1.0, 0.0),
                                  const float opacity = -1.0f
                                )
  {
    // Get symmetry plane parameters
    Eigen::Vector3f origin   = symmetry.getOrigin();
    Eigen::Vector3f normal  = symmetry.getNormal();
        
    // Generate a pose
    Eigen::Affine3f pose;
    pose.translation() = origin;
    pose.linear() = utl::alignVectors<float>(Eigen::Vector3f::UnitZ(), normal);
    
    // Display
    utl::showRectangle3d(visualizer, pose, side_width, side_width, id, color, opacity);
  }

  /** \brief Visualize a reflectional symmetry around a pointcloud. Symmetry is
   * visualized as a rectangle centered at the centroid of the pointcloud.
   * Rectangle width and height are chosen such that the pointcloud projected
   * onto the symmetry plane is bounded by the rectangle.
   *  \param[in]  visualizer  object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  symmetry    reflectional symmetry
   *  \param[in]  id          symmetry plane object id (default: symmetry)
   *  \param[in]  scale       scaling of the the rectangle (default 1.0)
   *  \param[in]  color       color of the displayed plane (default green)
   *  \param[in]  opacity     opacity of the displayed plane (default 1.0)
   */
  template <typename PointT>
  inline
  void showCloudReflectionalSymmetry  ( pcl::visualization::PCLVisualizer &visualizer,
                                        const typename pcl::PointCloud<PointT>::ConstPtr cloud,
                                        const ReflectionalSymmetry &symmetry,
                                        const std::string id = "symmetry",
                                        const float scale = 1.0f,
                                        utl::Color color = utl::Color(0.0, 1.0, 0.0),
                                        const float opacity = -1.0f
                                      )
  {
    // Get convex hull of the pointcloud projected on the symmetry plane
    typename pcl::PointCloud<PointT>::Ptr hullPoints (new pcl::PointCloud<PointT>);
    utl::ConvexHull2D<PointT> ch2d;
    ch2d.setInputCloud(cloud);
    ch2d.setPlaneCoefficients(symmetry.getPlaneCoefficients());
    ch2d.reconstruct(*hullPoints);
        
    // Ideally we would fit a 2D rectangle to the convex hull
    
    // Get the rotation of the symmetry rectangle
    Eigen::Affine3f pose;
    
    pcl::PCA<PointT> pca;
    pca.setInputCloud(hullPoints);
    Eigen::Vector3f cloudMean = pca.getMean().head(3);
    pose.linear().col(0) = pca.getEigenVectors().col(0);
    pose.linear().col(1) = pca.getEigenVectors().col(1);
    pose.linear().col(2) = pca.getEigenVectors().col(2);
    
    if (pose.linear().col(0).cross(pose.linear().col(1)).dot(pose.linear().col(2)) < 0)
      pose.linear().col(2) *= -1.0f;
                  
    // Get dimensions of the rectangle
    float widthMax = std::numeric_limits<float>::min();
    float widthMin = std::numeric_limits<float>::max();
    float heightMax = std::numeric_limits<float>::min();
    float heightMin = std::numeric_limits<float>::max();
    Eigen::Vector3f lineX_point = cloudMean + pose.linear().col(0);
    Eigen::Vector3f lineY_point = cloudMean + pose.linear().col(1);
    
    for (size_t pointId = 0; pointId < hullPoints->size(); pointId++)
    {
      Eigen::Vector3f pointProjected = hullPoints->points[pointId].getVector3fMap();
      
      float height = utl::pointToLineDistance<float>(pointProjected, cloudMean, lineX_point);
      if ((pointProjected - cloudMean).dot(lineY_point - cloudMean) < 0)
        height *= -1;
      heightMax = std::max(heightMax, height);
      heightMin = std::min(heightMin, height);
      
      float width = utl::pointToLineDistance<float>(pointProjected, cloudMean, lineY_point);
      if ((pointProjected - cloudMean).dot(lineX_point - cloudMean) < 0)
        width *= -1;      
      widthMax = std::max(widthMax, width);
      widthMin = std::min(widthMin, width);
    }

    float width = widthMax - widthMin;
    float height = heightMax - heightMin;    
    
    // Recenter the position of the symmetry rectangle    
    Eigen::Vector3f symmetryOriginVis = cloudMean;
    symmetryOriginVis += pose.linear().col(1) * (heightMax + heightMin) / 2.0f;
    symmetryOriginVis += pose.linear().col(0) * (widthMax + widthMin) / 2.0f;
    pose.translation() = symmetryOriginVis;    
    
    // Scale dimensions
//     if (width < height)
//       width = std::max(width, height / 2.0f);
//     else 
//       height = std::max(height, width / 2.0f);

    // Show symmetry
    utl::showRectangle3d(visualizer, pose, width*scale, height*scale, id,  color, opacity);
  }
  
  /** \brief Calculate the angle between a normal and a reflection of another
   * normal.
   *  \param[in]  normal1 first normal
   *  \param[in]  normal2 second normal
   *  \param[out] symmetry reflectional symmetry
   *  \param[in]  consistent_normals flag indicating if normals are consistently oriented (default true)
   *  \return angle (in radians) between two normals given the symmetry hypothesis
   */
  inline
  float getReflSymNormalFitError  ( const Eigen::Vector3f &normal1,
                                    const Eigen::Vector3f &normal2,
                                    const sym::ReflectionalSymmetry &symmetry,
                                    const bool &consisntent_normals = true
                                  )
  {
    Eigen::Vector3f normal2Reflected = symmetry.reflectNormal(normal2);
    float dotProd = utl::clampValue(normal1.dot(normal2Reflected), -1.0f, 1.0f);
    
    if (!consisntent_normals)
      dotProd = std::abs(dotProd);
    
    return std::acos(dotProd);
  }
  
  /** \brief Calculate the signed distance between the midpoint of two points
   * and a symmetry plane.
   *  \param[in] point1     first point
   *  \param[in] point2     second point
   *  \param[out] symmetry  reflectional symmetry
   *  \return position fit error
   */
  inline
  float getReflSymPositionFitError  ( const Eigen::Vector3f &point1,
                                      const Eigen::Vector3f &point2,
                                      const sym::ReflectionalSymmetry &symmetry
                                    )
  {
    Eigen::Vector3f midpoint = (point1 + point2) / 2.0f;
    return symmetry.pointSignedDistance(midpoint);
  }  
  
//   /** \brief Calculate the cosine of the angle between normal 1 and normal 2
//    * reflected by a symmetry hypothesis.
//    *  \param[in] normal1 first normal
//    *  \param[in] normal2 second normal
//    *  \param[out] symmetry reflectional symmetry
//    *  \param[in] consistent_normals flag indicating if normals are consistently oriented (default true)
//    *  \return angle (in radians) between two normals given the symmetry hypothesis
//    */
//   inline  
//   float getReflectionalSymmetryCorrespAngleCos  ( const Eigen::Vector3f &normal1,
//                                                   const Eigen::Vector3f &normal2,
//                                                   const sym::ReflectionalSymmetry &symmetry,
//                                                   const bool &consisntent_normals = true
//                                                 )
//   {
//     Eigen::Vector3f normal2Reflected = symmetry.reflectNormal(normal2);
//     float dotProd = utl::clampValue(normal1.dot(normal2Reflected), -1.0f, 1.0f);
//     
//     if (!consisntent_normals)
//       dotProd = std::abs(dotProd);
//     
//     return dotProd;
//   }
//   
//   /** \brief Calculate angle between normal 1 and normal 2 reflected by a
//    * symmetry hypothesis.
//    *  \param[in] normal1 first normal
//    *  \param[in] normal2 second normal
//    *  \param[out] symmetry reflectional symmetry
//    *  \param[in] consistent_normals flag indicating if normals are consistently oriented (default true)
//    *  \return angle (in radians) between two normals given the symmetry hypothesis
//    */
//   inline  
//   float getReflectionalSymmetryCorrespAngle ( const Eigen::Vector3f &normal1,
//                                               const Eigen::Vector3f &normal2,
//                                               const sym::ReflectionalSymmetry &symmetry,
//                                               const bool &consisntent_normals = true
//                                             )
//   {
//     float cosine = getReflectionalSymmetryCorrespAngleCos(normal1, normal2, symmetry, consisntent_normals);
//     return std::acos(cosine);
//   }
//   
//   /** \brief Calculate the angle symmetry plane and the line connecting the
//    * points of a correspondence.
//    *  \param[in] point1 first normal
//    *  \param[in] point2 second normal
//    *  \param[in] symmetry reflectional symmetry
//    *  \return angle (in radians) between the symmetry plane and the line connecting the points of a correspondence.
//    */
//   inline  
//   float getReflSymCorrPointError  ( const Eigen::Vector3f &point1,
//                                     const Eigen::Vector3f &point2,
//                                     const sym::ReflectionalSymmetry &symmetry
//                                   )
//   {
//     return utl::linePlaneAngle<float>((point2 - point1).normalized(), symmetry.getNormal());
//   }  
//   
//   /** \brief Calculate the orientation fit error for a symmetric correspondence.
//    *  \param[in] point1   first point
//    *  \param[in] normal1  first normal
//    *  \param[in] point2   second point
//    *  \param[in] normal2  second normal
//    *  \param[out] symmetry reflectional symmetry
//    *  \param[in] consistent_normals flag indicating if normals are consistently oriented (default true)
//    *  \return orientation fit error
//    */
//   inline
//   float getReflSymCorrOrientationFitError ( const Eigen::Vector3f &point1,
//                                             const Eigen::Vector3f &normal1,
//                                             const Eigen::Vector3f &point2,
//                                             const Eigen::Vector3f &normal2,
//                                             const sym::ReflectionalSymmetry &symmetry,
//                                             const bool &consisntent_normals = true
//                                           )
//   {
// //     float pointError  = getReflSymCorrPointError  (point1, point2, symmetry);
// //     float normalError = getReflSymCorrNormalError (normal1, normal2, symmetry, consisntent_normals);
// //         
// //     return normalError + pointError;
//     
//     // NOTE: it looks like using normal error alone results in better convergence
//     return getReflSymCorrNormalError (normal1, normal2, symmetry, consisntent_normals);    
//   }
  

  
//   /** \brief Calculate the orientation fir error for a symmetric correspondence.
//    *  \param[in] point1   first point
//    *  \param[in] normal1  first normal
//    *  \param[in] point2   second point
//    *  \param[in] normal2  second normal
//    *  \param[out] symmetry reflectional symmetry
//    *  \param[in] consistent_normals flag indicating if normals are consistently oriented (default true)
//    *  \return orientation fit error
//    */
//   template <typename PointT>
//   inline
//   float getReflSymCorrOrientationFitError ( const PointT &point1,
//                                             const PointT &point2,
//                                             const sym::ReflectionalSymmetry &symmetry,
//                                             const bool &consisntent_normals = true
//                                           )
//   {
//     float pointError  = getReflSymCorrPointError  (point1.getVector3fMap(), point2.getVector3fMap(), symmetry);
//     float normalError = getReflSymCorrNormalError (point1.getNormalVector3fMap(), point2.getNormalVector3fMap(), symmetry, consisntent_normals);
//     return pointError + normalError;
//   }  
}

#endif // REFLECTIONAL_SYMMETRY_HPP