// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef ROTATIONAL_SYMMETRY_HPP
#define ROTATIONAL_SYMMETRY_HPP

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Utilities includes
#include <geometry/geometry.hpp>
#include <pointcloud/pointcloud.hpp>
#include <visualization/pcl_visualization.hpp>

namespace sym
{  
  /** \brief Class representing a rotational symmetry in 3D space. Symmetry is
   * represented as a 3D axis.
   */  
  class RotationalSymmetry
  {
  public:
    
    /** \brief An empty constructor */
    RotationalSymmetry ()
      : origin_ (Eigen::Vector3f::Zero())
      , direction_ (Eigen::Vector3f::Zero())
    {};
    
    /** \brief A constructor from origin point and direction
     *  \param[in] origin origin point
     *  \param[in] direction direction
     */
    RotationalSymmetry (const Eigen::Vector3f &origin, const Eigen::Vector3f &direction)
      : origin_ (origin)
      , direction_ (direction.normalized())
    { };
    
    /** \brief Get the direction vector of the symmetry axis
     *  \return direction vector
     */    
    Eigen::Vector3f getDirection () const { return direction_; }
    
    /** \brief Get the origin point of the symmetry
     *  \return symmetry origin point
     */            
    Eigen::Vector3f getOrigin () const { return origin_; }
            
    /** \brief Set the origin point of the symmetry
     *  \param[in] origin symmetry origin point
     */
    void setOrigin  (const Eigen::Vector3f origin ) { origin_  = origin;  }
    
    /** \brief Set the direction vector of the symmetry
     *  \param[in] direction symmetry direction
     */
    void setDirection  (const Eigen::Vector3f direction) { direction_ = direction.normalized(); }
    
    /** \brief Project a 3D point on the symmetry axis
     *  \param[in] point  point to be projected
     *  \return projected point
     */    
    Eigen::Vector3f projectPoint  (const Eigen::Vector3f &point)  const
    {
      return utl::projectPointToLine<float>(point, origin_, origin_ + direction_);
    };
    
    /** \brief Set the origin of the symmetry to be the input point projected on the current symmetry axis
     *  \param[in] point  point who's projection will be used as new origin
     */
    void setOriginProjected  (const Eigen::Vector3f &point)
    {
      origin_ = projectPoint(point);
    };

    /** \brief Set symmetry parameters from a point and a direction
     *  \param[in] point      origin point
     *  \param[in] direction  direction vector (can be non-unit)
     */
    void fromPointDirection (const Eigen::Vector3f &point, const Eigen::Vector3f &direction)
    {
      origin_ = point;
      direction_ = direction.normalized();
    }
    
    /** \brief Set symmetry parameters from two points
     *  \param[in] point1  first point
     *  \param[in] point2  second point
     */
    void fromTwoPoints (const Eigen::Vector3f &point1, const Eigen::Vector3f &point2)
    {
      fromPointDirection(point1, point2 - point1);
    }

    /** \brief Set symmetry parameters from two points
     *  \param[in] points  6D vector where first three entries correspond to first point and second 3 entries to second point
     */
    void fromTwoPoints (const Eigen::Matrix<float, 6, 1> &points)
    {
      fromTwoPoints(points.head(3), points.tail(3));
    }
    
    /** \brief Transform the symmetry axis with an 3D rigid transformation.
     *  \param[in]  transform 3D affine transform
     */
    inline
    RotationalSymmetry transform  ( const Eigen::Affine3f &transform  ) const
    {
      return RotationalSymmetry(transform * origin_, transform.rotation() * direction_);
    }    

    /** \brief Write symmetry parameters to a filestream in ASCII format.
     *  \param[in] file output filestream
     */
    void writeASCII (std::ofstream &file) const
    {
      file << "origin:\n";
      file << "   " << origin_[0] << " " << origin_[1] << " " << origin_[2] << "  " << std::endl;
      file << "direction:\n";
      file << "   " << direction_[0] << " " << direction_[1] << " " << direction_[2] << std::endl;
    }
    
    /** \brief Read symmetry parameters from an ASCII file. Filestream pointer
     * must point to a line starting a valid symmetry record.
     *  \param[in] file input filestream
     */
    bool readASCII (std::ifstream &filestream)
    {
      std::string line;
      std::istringstream iss;
            
      // Read point
      std::getline(filestream,line);
      if (line != "origin:")
      {
        std::cout << "[sym::RotationalSymmetry::readASCII] 'origin:' line does not match the expected format (" << line << ")." << std::endl;
        return false;
      }
      std::getline(filestream,line);
      iss.str(line);
      iss >> origin_[0] >> origin_[1] >> origin_[2];

      // Read direction
      std::getline(filestream,line);
      if (line != "direction:")
      {
        std::cout << "[sym::RotationalSymmetry::readASCII] 'direction:' line does not match the expected format  (" << line << ")." << std::endl;
        return false;
      }
      std::getline(filestream,line);
      iss.str(line);
      iss >> direction_[0] >> direction_[1] >> direction_[2];
      
      return true;
    }    

    /** \brief Get the distance between a symmetry axis and a point.
     *  \param[in] point  point
     *  \return distance
     */    
    float pointDistance (const Eigen::Vector3f &point)  const
    {
      return utl::pointToLineDistance<float>(point, origin_, origin_ + direction_);
    };
    
//     /** \brief Calculate the angle and distance between two symmetry axes.
//      *  \param[in]  symmetry_other second symmetry
//      *  \param[out]  distance  distance between symmetry axes
//      *  \param[out]  angle     angle between symmetry axes
//      *  \note maximum angle between two symmetry axes is 90 degrees
//      */
//     void difference (const RotationalSymmetry &symmetry_other, float &distance, float &angle) const
//     {
//       distance  = utl::lineToLineDistance<float>(origin_, origin_+direction_, symmetry_other.getOrigin(),  symmetry_other.getOrigin()+symmetry_other.getDirection());
//       angle     = utl::lineLineAngle<float>(origin_, origin_+direction_, symmetry_other.getOrigin(), symmetry_other.getOrigin()+symmetry_other.getDirection());
// //       angle     = std::min(angle, static_cast<float>(M_PI) - angle);
//       return;
//     }

    /** \brief Calculate the difference between two rotational symmetries. Two
     * measures are calculated:
     * 1) Angle between symmetry axes
     * 2) Distance between the projections of a reference point onto the two
     *    symmetry axes
     *  \param[in]  symmetry_other second symmetry
     *  \param[in]  reference_point reference point
     *  \param[out]  angle     angle between symmetry axes
     *  \param[out]  distance  distance between projections of the reference point
     *  \note maximum angle between two symmetry axes is 90 degrees
     */
    void rotSymDifference (const RotationalSymmetry &symmetry_other, const Eigen::Vector3f &reference_point, float &angle, float &distance) const
    {
      angle     = utl::lineLineAngle<float>(origin_, origin_+direction_, symmetry_other.getOrigin(), symmetry_other.getOrigin()+symmetry_other.getDirection());
      
      Eigen::Vector3f refPointProj1 = projectPoint(reference_point);
      Eigen::Vector3f refPointProj2 = symmetry_other.projectPoint(reference_point);
      distance = utl::pointToPointDistance<float>(refPointProj1, refPointProj2);

      return;
    }    
    
    /** \brief Get a rotation matrix for rotating around the symmetry axis by a
     * specified angle.
     *  \param[in] angle angle of rotation in radians
     */
    Eigen::Matrix3f getRotationAroundAxis (const float angle) const
    {
      return Eigen::AngleAxisf(angle, direction_ ).toRotationMatrix();
    }
    
    /** \brief Rotate a point around a symmetry axis by a given rotation matrix.
     * This is equivalent to rotating a vector from symmetry axis to the point 
     * by a given rotation matrix.
     *  \param[in] point point to be rotated
     *  \param[in] R rotation matrix
     */
    Eigen::Vector3f rotatePoint  (const Eigen::Vector3f &point, const Eigen::Matrix3f R) const
    {   
      Eigen::Vector3f pointProjected = projectPoint(point);
      return pointProjected + R * (point - pointProjected);
    }

    /** \brief Rotate a point around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] point point to be rotated
     *  \param[in] angle angle of rotation in radians
     */
    Eigen::Vector3f rotatePoint  (const Eigen::Vector3f &point, const float angle) const
    {   
      return rotatePoint(point, getRotationAroundAxis(angle));
    }

    /** \brief Rotate a normal vector by a given rotation matrix.
     *  \param[in] normal normal to be rotated
     *  \param[in] angle angle of rotation in radians
     */
    Eigen::Vector3f rotateNormal  (const Eigen::Vector3f &normal, const Eigen::Matrix3f R) const
    {
      return R * normal;
    }
    
    /** \brief Rotate a normal around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] normal normal to be rotated
     *  \param[in] angle angle of rotation in radians
     */
    Eigen::Vector3f rotateNormal  (const Eigen::Vector3f &normal, const float angle) const
    {
      return rotateNormal(normal, getRotationAroundAxis(angle));
    }
    
    /** \brief Rotate a pointcloud around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out rotated cloud
     *  \param[in] angle angle of rotation in radians
     */
    template <typename PointT>
    void rotateCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, const float angle) const
    {
      // Prepare output cloud
      pcl::copyPointCloud<PointT>(cloud_in, cloud_out);
      
      for (size_t i = 0; i < cloud_in.size(); i++)
        cloud_out.points[i].getVector3fMap() = rotatePoint(cloud_in.points[i].getVector3fMap(), angle);
    }
    
    /** \brief Rotate a pointcloud around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] cloud_in original cloud
     *  \param[in] indices  indices of the points that need to be rotated
     *  \param[in] cloud_out rotated cloud
     *  \param[in] angle angle of rotation in radians
     */
    template <typename PointT>
    void rotateCloud(const pcl::PointCloud<PointT> &cloud_in, const std::vector<int> &indices, pcl::PointCloud<PointT> &cloud_out, const float angle) const
    {
      // Prepare output cloud
      pcl::copyPointCloud<PointT>(cloud_in, indices, cloud_out);
      
      for (size_t i = 0; i < indices.size(); i++)
        cloud_out.points[i].getVector3fMap() = rotatePoint(cloud_in.points[indices[i]].getVector3fMap(), angle);
    }

    /** \brief Rotate a pointcloud with normals around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] cloud_in original cloud
     *  \param[in] cloud_out rotated cloud
     *  \param[in] angle angle of rotation in radians
     */
    template <typename PointT>
    void rotateCloudWithNormals ( const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, const float angle)  const
    {
      pcl::copyPointCloud<PointT>(cloud_in, cloud_out);
          
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        Eigen::Matrix3f R = getRotationAroundAxis(angle);
        cloud_out.points[i].getVector3fMap()        = rotatePoint  (cloud_in.points[i].getVector3fMap(), R);
        cloud_out.points[i].getNormalVector3fMap()  = rotateNormal (cloud_in.points[i].getNormalVector3fMap(), R);
      }
    }

    /** \brief Rotate a pointcloud with normals around a symmetry axis by a given angle. The 
     * angle is specified clockwise around the symmetry axis.
     *  \param[in] cloud_in original cloud
     *  \param[in] indices  indices of the points that need to be rotated
     *  \param[in] cloud_out rotated cloud
     *  \param[in] angle angle of rotation in radians
     */
    template <typename PointT>
    void rotateCloudWithNormals ( const pcl::PointCloud<PointT> &cloud_in, const std::vector<int> &indices, pcl::PointCloud<PointT> &cloud_out, const float angle)  const
    {
      pcl::copyPointCloud<PointT>(cloud_in, indices, cloud_out);
          
      for (size_t i = 0; i < cloud_in.size(); i++)
      {
        Eigen::Matrix3f R = getRotationAroundAxis(angle);
        cloud_out.points[i].getVector3fMap()        = rotatePoint  (cloud_in.points[indices[i]].getVector3fMap(), R);
        cloud_out.points[i].getNormalVector3fMap()  = rotateNormal (cloud_in.points[indices[i]].getNormalVector3fMap(), R);
      }
    }
    
    /** \brief Get the normal of the plane used to construct the profile curve.
     * The normal is constructed as unit direction vector of a line that goes
     * through the coordinate system origin and is perpendicular to the symmetry
     * axis.
     *  \return normal of the profile curve plane
     */
    Eigen::Vector3f getProfileCurvePlaneNormal (  )  const
    {
//       Eigen::Vector3f coordinateOriginProjected = utl::projectPointToLine<float>(Eigen::Vector3f::Zero(), origin_, origin_ + direction_);
      Eigen::Vector3f coordinateOriginProjected = projectPoint(Eigen::Vector3f::Zero());
      return (Eigen::Vector3f::Zero()-coordinateOriginProjected).normalized();
    }
        
    /** \brief Get the profile curve of a cloud under current symmetry axis.
     *  \param[in]  cloud input cloud
     *  \param[out] profile_curve profile curve
     */
    template <typename PointT>
    void getProfileCurve ( const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &profile_curve)  const
    {
      // Get a vector in the profile curve plane that is perpendicular to the symmetry axis
      Eigen::Vector3f planeNormal = getProfileCurvePlaneNormal();
      Eigen::Vector3f planeVector = direction_.cross(planeNormal);
      
      // Rotate all cloud points onto that plane
      profile_curve.resize(cloud.size());
      for (size_t pointId = 0; pointId < profile_curve.size(); pointId++)
      {
        Eigen::Vector3f point = cloud.points[pointId].getVector3fMap();
//         Eigen::Vector3f pointProjected = utl::projectPointToLine<float>(point, origin_, origin_ + direction_);
        Eigen::Vector3f pointProjected = projectPoint(point);
        float distance = (point - pointProjected).norm();
        Eigen::Vector3f pointRotated = pointProjected + planeVector * distance;
        profile_curve.points[pointId].getVector3fMap() = pointRotated;
      }
    }
    
    /** \brief Get the profile curve of a cloud under current symmetry axis.
     *  \param[in]  cloud input cloud
     *  \param[out] profile_curve profile curve
     *  \note 3 times slower than getProfileCurve
     */
    template <typename PointT>
    void getProfileCurveWithNormals ( const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &profile_curve)  const
    {
      // Get a vector in the profile curve plane that is perpendicular to the symmetry axis
      Eigen::Vector3f planeNormal = getProfileCurvePlaneNormal();
      Eigen::Vector3f planeVector = direction_.cross(planeNormal);
      
      // Rotate all cloud points onto that plane
      profile_curve.resize(cloud.size());
      for (size_t pointId = 0; pointId < profile_curve.size(); pointId++)
      {
        Eigen::Vector3f point = cloud.points[pointId].getVector3fMap();
        Eigen::Vector3f normal = cloud.points[pointId].getNormalVector3fMap();
        
        // Rotate point
        Eigen::Vector3f pointProjectedOnAxis = projectPoint(point);
        float pointToAxisDistance = (point - pointProjectedOnAxis).norm();
        Eigen::Vector3f pointRotated = pointProjectedOnAxis + planeVector * pointToAxisDistance;
        
        // Rotate normal
        Eigen::Matrix3f F1;   // Coordinate system associated with normal
        F1.col(0) = direction_;
        F1.col(1) = (point - pointProjectedOnAxis).normalized();
        F1.col(2) = F1.col(0).cross(F1.col(1));
        
        Eigen::Matrix3f F2;   // Coordinate system associated with normal
        F2.col(0) = direction_;
        F2.col(1) = (pointRotated - pointProjectedOnAxis).normalized();
        F2.col(2) = F2.col(0).cross(F2.col(1));
        
        Eigen::Vector3f normal_F1 = F1.transpose() * normal;
        Eigen::Vector3f normalRotated = F2 * normal_F1;
        
        // Assign
        profile_curve.points[pointId].getVector3fMap() = pointRotated;
        profile_curve.points[pointId].getNormalVector3fMap() = normalRotated;        
      }
    }    
        
    /** \brief Reconstruct a rotationaly symmetric pointcloud model from a pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[in]  indices  indices of the points that need to be rotated
     *  \param[out] cloud_reconstructed reconstructed cloud
     *  \param[in]  angular_step  angular step between rotations of the profile step
     */
    template <typename PointT>
    void reconstructCloud ( const typename pcl::PointCloud<PointT> &cloud,
                            const std::vector<int> &indices,
                            typename pcl::PointCloud<PointT> &cloud_reconstructed,
                            const float angular_step = M_PI/4
                          ) const
    {
      cloud_reconstructed.clear();
      int numIters = static_cast<int>(M_PI * 2 / angular_step) + 1;
      
      pcl::PointCloud<PointT> cloudRotated;
      for (size_t i = 0; i < numIters; i++)
      {
        this->rotateCloud<PointT>(cloud, indices, cloudRotated, angular_step * i);
        cloud_reconstructed += cloudRotated;
      }
    }

    /** \brief Reconstruct a rotationaly symmetric pointcloud model from a pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[out] cloud_reconstructed reconstructed cloud
     *  \param[in]  angular_step  angular step between rotations of the profile step
     */
    template <typename PointT>
    void reconstructCloud ( const typename pcl::PointCloud<PointT> &cloud,
                            typename pcl::PointCloud<PointT> &cloud_reconstructed,
                            const float angular_step = M_PI/4
                          ) const
    {
      // Create fake indices
      std::vector<int> indices (cloud.size());
      for (size_t pointId = 0; pointId < cloud.size(); pointId++)
        indices[pointId] = pointId;
      
      // Reconstruct
      reconstructCloud<PointT>(cloud, indices, cloud_reconstructed, angular_step);
    }
    
    /** \brief Reconstruct a rotationaly symmetric pointcloud model from a pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[in]  indices  indices of the points that need to be rotated
     *  \param[out] cloud_reconstructed reconstructed cloud
     *  \param[in]  angular_step  angular step between rotations of the profile step
     */
    template <typename PointT>
    void reconstructCloudWithNormals  ( const typename pcl::PointCloud<PointT> &cloud,
                                        const std::vector<int> &indices,
                                        typename pcl::PointCloud<PointT> &cloud_reconstructed,
                                        const float angular_step = M_PI/4
                                      ) const
    {
      cloud_reconstructed = cloud;
      int numIters = static_cast<int>(M_PI * 2 / angular_step) + 1;
      
      pcl::PointCloud<PointT> cloudRotated;
      for (size_t i = 0; i < numIters; i++)
      {
        this->rotateCloudWithNormals<PointT>(cloud, indices, cloudRotated, angular_step * i);
        cloud_reconstructed += cloudRotated;
      }
    }

    /** \brief Reconstruct a rotationaly symmetric pointcloud model from a pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[out] cloud_reconstructed reconstructed cloud
     *  \param[in]  angular_step  angular step between rotations of the profile step
     */
    template <typename PointT>
    void reconstructCloudWithNormals  ( const typename pcl::PointCloud<PointT> &cloud,
                                        typename pcl::PointCloud<PointT> &cloud_reconstructed,
                                        const float angular_step = M_PI/4
                                      ) const
    {
      // Create fake indices
      std::vector<int> indices (cloud.size());
      for (size_t pointId = 0; pointId < cloud.size(); pointId++)
        indices[pointId] = pointId;
      
      // Reconstruct
      reconstructCloudWithNormals<PointT>(cloud, indices, cloud_reconstructed, angular_step);
    }    
    
    /** \brief Get cylinder centered on a symmetry axis that bounds the
     * pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[in]  indices indices of the points of the cloud that are used
     *  \param[out] cylinder_coeffs coefficients of a bounding cylinder (as defined in PCL)
     *  \return     TRUE if a cylinder was fitted successfully
     */
    template <typename PointT>
    bool getBoundingCylinder  ( const pcl::PointCloud<PointT> &cloud,
                                const std::vector<int> &indices,
                                pcl::ModelCoefficients  &cylinder_coeffs
                              ) const
    {
      cylinder_coeffs.values.resize(7);
      
      // Check that input cloud is not empty
      if (cloud.size() == 0)
      {
        std::cout << "[sym::RotationalSymmetry::getBoundingCylinder] input cloud is empty!" << std::endl;
        cylinder_coeffs.values[6] = -1.0f;
        return false;
      }
      
      std::vector<float> alphas (indices.size());
      float radius = 0.0f;
      for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
      {
        int pointId = indices[pointIdIt];
        
        Eigen::Vector3f point = cloud.points[pointId].getVector3fMap();
        Eigen::Vector3f pointProjected = projectPoint(point);
        radius = std::max(radius, (point - pointProjected).norm());
        alphas[pointIdIt] = direction_.dot(pointProjected - origin_);
      }
      
      float alphaMax = utl::vectorMax(alphas);
      float alphaMin = utl::vectorMin(alphas);
      
      Eigen::Vector3f cylinderOrigin    = origin_ + direction_ * alphaMin;
      Eigen::Vector3f cylinderDirection = direction_ * (alphaMax - alphaMin);
      
      cylinder_coeffs.values[0]  = cylinderOrigin[0];
      cylinder_coeffs.values[1]  = cylinderOrigin[1];
      cylinder_coeffs.values[2]  = cylinderOrigin[2];
      cylinder_coeffs.values[3]  = cylinderDirection[0];
      cylinder_coeffs.values[4]  = cylinderDirection[1];
      cylinder_coeffs.values[5]  = cylinderDirection[2];
      cylinder_coeffs.values[6]  = radius;
      
      return true;
    }
    
    /** \brief Get cylinder centered on a symmetry axis that bounds the
     * pointcloud.
     *  \param[in]  cloud input cloud
     *  \param[in]  indices indices of the points of the cloud that are used
     *  \param[out] cylinder_coeffs coefficients of a bounding cylinder (as defined in PCL)
     *  \return     TRUE if a cylinder was fitted successfully
     */
    template <typename PointT>
    void getBoundingCylinder  ( const pcl::PointCloud<PointT> &cloud,
                                pcl::ModelCoefficients  &cylinder_coeffs
                              ) const
    {
      // Create fake indices
      std::vector<int> indices (cloud.size());
      for (size_t pointId = 0; pointId < cloud.size(); pointId++)
        indices[pointId] = pointId;
      
      return getBoundingCylinder<PointT>(cloud, indices, cylinder_coeffs);
    }
    
  protected:
    
    // Member variables
    Eigen::Vector3f origin_;  ///< Point belonging to the symmetry axis. Symmetry is visualized around this point.
    Eigen::Vector3f direction_;  ///< Unit length vector representing the direction of the symmetry axis.
  };
  
  /** \brief Print symmetry details to ostream */
  inline
  std::ostream& operator<< ( std::ostream& os, const RotationalSymmetry& symmetry)
  {
    os << "origin:    " << symmetry.getOrigin().transpose();
    os << std::endl;
    os << "direction: " << symmetry.getDirection().transpose();
      
    return os;
  }  
  
  /** \brief Visualize a rotational symmetry as a line segment centered at the symmetry origin point
   *  \param[in] visualizer object
   *  \param[in] symmetry rotational symmetry
   *  \param[in] id symmetry axis object id (default: symmetry)
   *  \param[in] length length of the line segment
   *  \param[in] line_width line segment width
   *  \param[in] color color of the line segment
   *  \param[in] opacity of the line segment
   */
  inline
  void showRotationalSymmetry ( pcl::visualization::PCLVisualizer &visualizer,
                                const RotationalSymmetry &symmetry,
                                const std::string id = "symmetry",
                                float length = 0.2,
                                float line_width = 2.0,
                                utl::Color color = utl::green,
                                float opacity = -1.0f
                              )
  {
    pcl::PointXYZ p1, p2;
    p1.getVector3fMap() = symmetry.getOrigin() + symmetry.getDirection() * length / 2;
    p2.getVector3fMap() = symmetry.getOrigin() - symmetry.getDirection() * length / 2;
    
    visualizer.addLine(p1, p2, id);
    utl::setLineRenderProps(visualizer, id, line_width, color, opacity);
  }
  
  /** \brief Visualize a rotational symmetry around a pointcloud. Symmetry is
   * visualized as a line with endpoints obtained by projecting the pointcloud
   * points onto the symmetry line.
   *  \param[in]  visualizer  object
   *  \param[in]  cloud       pointcloud
   *  \param[in]  symmetry    rotational symmetry
   *  \param[in]  id          symmetry plane object id (default: symmetry)
   *  \param[in]  length      the length to which the line extends outside the pointcloud bounds
   *  \param[in]  line_width  line segment width
   *  \param[in]  color       color of the displayed plane (default green)
   *  \param[in]  opacity     opacity of the displayed plane (default 1.0)
   */
  template <typename PointT>
  inline
  void showCloudRotationalSymmetry  ( pcl::visualization::PCLVisualizer &visualizer,
                                      const typename pcl::PointCloud<PointT> cloud,
                                      const RotationalSymmetry &symmetry,
                                      const std::string id = "symmetry",
                                      const float length = 1.0f,
                                      const float line_width = 1.0f,
                                      utl::Color color = utl::Color(0.0, 1.0, 0.0),
                                      const float opacity = -1.0f
                                    )
  {
    // Project cloud to the symmetry axis and figure out the furthest points
    float distMax = std::numeric_limits<float>::min();
    float distMin = std::numeric_limits<float>::max();
    Eigen::Vector3f pointMax, pointMin;
    
    for (size_t pointId = 0; pointId < cloud.size(); pointId++)
    {
      Eigen::Vector3f point = cloud.points[pointId].getVector3fMap();
      Eigen::Vector3f pointProjected = utl::projectPointToLine<float>(point, symmetry.getOrigin(), symmetry.getOrigin() + symmetry.getDirection());
      
      float distance = utl::pointToPointDistance<float>(pointProjected, symmetry.getOrigin());
      if ((pointProjected - symmetry.getOrigin()).dot(symmetry.getDirection()) < 0)
        distance *= -1;
      
      if (distance > distMax)
      {
        distMax = distance;
        pointMax = pointProjected;
      }
      
      if (distance < distMin)
      {
        distMin = distance;
        pointMin = pointProjected;
      }
    }
    
    // Scale points
    pointMax += symmetry.getDirection() * length;
    pointMin -= symmetry.getDirection() * length;
    
    // Show symmetry
    pcl::PointXYZ p1, p2;
    p1.getVector3fMap() = pointMax;
    p2.getVector3fMap() = pointMin;
    
    visualizer.addLine(p1, p2, id);
    utl::setLineRenderProps(visualizer, id, line_width, color, opacity);
  }  
  
  /** \brief Calculate the error of fit between a symmetry axis and an oriented
    * point. Error of fit is calculated as the angle between the point normal
    * and the plane containing the symmetry axis and the point itself.
    *  \param[in]  point                   point coordinate
    *  \param[in]  normal                  point normal
    *  \param[in]  symmetry                symmetry axis
    *  \return angle (in radians) between the point normal and the plane formed by the symmetry axis and the point
    */
  inline
  float getRotSymFitError ( const Eigen::Vector3f     &point,
                            const Eigen::Vector3f     &normal,
                            const RotationalSymmetry  &symmetry
                          )
  {
    Eigen::Vector3f pointProjected = symmetry.projectPoint (point);
    Eigen::Vector3f planeNormal = (point - pointProjected).cross(symmetry.getDirection());
   
    // Calculate angle sine
    float angle_sin = std::abs (planeNormal.dot(normal) / planeNormal.norm());
    angle_sin = utl::clampValue(angle_sin, 0.0f, 1.0f);
    
    // Limit angle sine and return
    return std::asin(angle_sin);
  }
  
  /** \brief Calculate how perpendicular a surface point is to the symmetry
   * axis. Perpendicularity is calculated as:
   *   1 - (angle between symmetry axis and point normal / PI/2)
   * The final score is in the [0, 1] range. Higher values indicate higher
   * perpendicularity. Optinonally the score can be clamped to a certain maximum
   * angle between the symmetry axis and point normal (the score will still be 
   * normalized to the [0, 1] range).
   *  \param[in]  normal          point normal
   *  \param[in]  symmetry        input symmetry
   *  \param[in]  angle_threshold angle threshold used for clamping
   *  \return point perpendicularity score
   */
  inline
  float getRotSymPerpendicularity ( const Eigen::Vector3f normal,
                                    const sym::RotationalSymmetry &symmetry,
                                    const float angle_threshold = M_PI / 2
                                  )
  {
    if (angle_threshold == 0.0f)
      return 1.0f;
    
    // Get perpendicular score
    float symAxisNormalAngle = utl::lineLineAngle(symmetry.getDirection(), normal);

    // Normalize by angle threshold
    symAxisNormalAngle /= angle_threshold;
    symAxisNormalAngle = std::min (symAxisNormalAngle, 1.0f);
    
    return 1.0f - symAxisNormalAngle;
  }
}

#endif // ROTATIONAL_SYMMETRY_HPP