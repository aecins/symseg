// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GEOMETRY_UTILITIES_HPP
#define GEOMETRY_UTILITIES_HPP

#include <iostream>
#include <geometry/math.hpp>

namespace utl
{
  //--------------------------------------------------------------------------
  // Conversions
  //--------------------------------------------------------------------------
  
  /** \brief Extract plane unit normal and point on the plane closest to the origin
    * from coefficients of an equation of a plane (ax + by + cz + d = 0).
    *  \param[in]  plane_coefficients plane coefficients (ax + by + cz + d = 0)
    *  \param[out] plane_point plane point closest to the origin
    *  \param[out] plane_normal plane unit normal
    */
  template <class Scalar>
  inline
  void planeCoefficientsToPointNormal (const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients, Eigen::Matrix< Scalar, 3, 1> &plane_point, Eigen::Matrix< Scalar, 3, 1> &plane_normal)
  {
    plane_normal << plane_coefficients[0], plane_coefficients[1], plane_coefficients[2];
    Scalar norm = plane_normal.norm();
    plane_normal /= norm;
    plane_point  = plane_normal * ( - plane_coefficients[3] / norm );
  }

  /** \brief Given a point an a normal defining a plane extract coefficients 
    * of the equation of a plane (ax + by + cz + d = 0).
    *  \param[in]  plane_point plane point closest to the origin
    *  \param[in]  plane_normal plane unit normal
    *  \param[out] plane_coefficients plane coefficients (ax + by + cz + d = 0)
    */
  template <class Scalar>
  inline void
  pointNormalToPlaneCoefficients (const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal, Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
  {
    plane_coefficients.head(3) = plane_normal;
    plane_coefficients(3) = - plane_normal.dot(plane_point);
  }

  //--------------------------------------------------------------------------
  // Distances
  //--------------------------------------------------------------------------    
  
  /** \brief Get the Euclidean distance between two N-dimensional points.
    *  \param[in] point1 first point
    *  \param[in] point2 second point
    *  \return distance between points
    */
  template <class Scalar>
  inline Scalar
  pointToPointDistance (const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> &point1, const Eigen::Matrix< Scalar, Eigen::Dynamic, 1> &point2)
  {
    if (point1.size() != point2.size())
    {
      std::cout << "[utl::geom::pointToPointDistance] points have different dimensions. Returning NaN." << std::endl;
      return std::numeric_limits<float>::quiet_NaN();
    }
    return (point1-point2).norm();
  }
  
  /** \brief Get distance between a point and a line.
    *  \param[in] point point
    *  \param[in] line_point1  first point of a line
    *  \param[in] line_point2  second point of a line
    *  \return distance between point and line
    */
  template <class Scalar>
  inline Scalar
  pointToLineDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2)
  {
    return ((point - line_point1).cross(point - line_point2)).norm() / (line_point2 - line_point1).norm();
  }
  
  /** \brief Get signed distance between a point and a plane.
    *  \param[in] point point
    *  \param[in] plane_point  a point on the plane
    *  \param[in] plane_normal plane normal
    *  \return signed distance between point and plane
    */
  template <class Scalar>
  inline Scalar
  pointToPlaneSignedDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal)
  {
    Eigen::Matrix< Scalar, 3, 1> planeToPointVector = point - plane_point;
    return planeToPointVector.dot(plane_normal);
  }
  
  /** \brief Get the signed distance between a point and a plane.
    *  \param[in] point point
    *  \param[in] plane_coefficients coefficients of the equation of the plane
    *  \return signed distance between point and plane
    */
  template <class Scalar>
  inline
  Scalar pointToPlaneSignedDistance (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
  {
    Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
    planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);
    return pointToPlaneSignedDistance<Scalar>(point, plane_point, plane_normal);
  }

  /** \brief Get distance between two skew lines.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] line2_point1 first point of the line 2
    *  \param[in] line2_point2 second point of the line 2
    *  \param[in] eps  tolerance for parallel lines check
    *  \return distance between two lines
    *  \note http://mathworld.wolfram.com/Line-LineDistance.html
    */
  template <class Scalar>
  inline
  Scalar lineToLineDistance ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                              const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                              const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                              const Eigen::Matrix< Scalar, 3, 1> &line2_point2,
                              const Scalar eps = 1e-12
                            )
  {
    // Get line direction vectors
    Eigen::Vector3f a = line1_point2 - line1_point1;
    Eigen::Vector3f b = line2_point2 - line2_point1;
    
    // If lines are parallel return the distance between point and line
    Scalar denom = a.cross(b).norm();
    if (denom < eps)
      return pointToLineDistance<Scalar>(line2_point1, line1_point1, line1_point2);
    
    Eigen::Vector3f c = line2_point1 - line1_point1;
    return std::abs(c.dot(a.cross(b))) / denom;
  }

  //--------------------------------------------------------------------------
  // Angles
  //--------------------------------------------------------------------------

  /** \brief Get cosine of the angle between two vectors.
    *  \param[in] v1 vector 1 (must be unit length)
    *  \param[in] v2 vector 2 (must be unit length)
    *  \return cosine of the angle between the input vectors
    */
  template <class Scalar>
  inline
  Scalar vectorVectorAngleCos ( const Eigen::Matrix< Scalar, 3, 1> &v1,
                                const Eigen::Matrix< Scalar, 3, 1> &v2
                              )
  {
    return utl::clampValue(v1.dot(v2), -1.0f, 1.0f);
  }

  /** \brief Get the angle between two vectors.
    *  \param[in] v1 vector 1 (must be unit length)
    *  \param[in] v2 vector 2 (must be unit length)
    *  \return cosine of the angle between the input vectors
    */
  template <class Scalar>
  inline
  Scalar vectorVectorAngle  ( const Eigen::Matrix< Scalar, 3, 1> &v1,
                              const Eigen::Matrix< Scalar, 3, 1> &v2
                            )
  {
    return std::acos(vectorVectorAngleCos<Scalar>(v1, v2));
  }
  
  /** \brief Find a clockwise angle between two 3D vectors
    *  \param[in] v1 first vector
    *  \param[in] v2 second vector
    *  \param[in] normal plane normal
    *  \return clockwise angle between two vectors
    *  \note assumes right handed coordinate system
    */
  template <typename Scalar>
  inline Scalar
  vectorVectorAngleCW (const Eigen::Matrix< Scalar, 3, 1> &v1, const Eigen::Matrix< Scalar, 3, 1> &v2, const Eigen::Matrix< Scalar, 3, 1> &normal)
  {
    Scalar cos = v1.dot(v2);
    Scalar sin = utl::clampValue<Scalar> (normal.dot (v1.cross (v2)), -1.0, 1.0);
    return std::atan2 (sin, cos);
  }

  /** \brief Get the cosine of the angle between two skew lines.
    *  \param[in] line1_direction line 1 direction vector (must be unit length)
    *  \param[in] line2_direction line 2 direction vector (must be unit length)
    *  \return angle between input lines
    */
  template <class Scalar>
  inline
  Scalar lineLineAngleCos ( const Eigen::Matrix< Scalar, 3, 1> &line1_direction,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_direction
                          )
  {
    return std::abs(vectorVectorAngleCos<Scalar>(line1_direction, line2_direction));
  }

  /** \brief Calculate the angle between two skew lines.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] line2_point1 first point of the line 2
    *  \param[in] line2_point2 second point of the line 2
    *  \return angle between two lines
    */
  template <class Scalar>
  inline
  Scalar lineLineAngleCos ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                            const Eigen::Matrix< Scalar, 3, 1> &line2_point2
                          )
  {
    // Get line direction vectors
    Eigen::Vector3f a = (line1_point2 - line1_point1).normalized();
    Eigen::Vector3f b = (line2_point2 - line2_point1).normalized();
    
    return lineLineAngleCos<Scalar>(a, b);
  }
  
  /** \brief Get angle between two skew lines.
    *  \param[in] line1_direction line 1 direction vector (must be unit length)
    *  \param[in] line2_direction line 2 direction vector (must be unit length)
    *  \return angle between input lines
    */
  template <class Scalar>
  inline
  Scalar lineLineAngle  ( const Eigen::Matrix< Scalar, 3, 1> &line1_direction,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_direction
                        )
  {
    return std::acos(lineLineAngleCos<Scalar>(line1_direction, line2_direction));
  }
  
  
  /** \brief Calculate the angle between two skew lines.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] line2_point1 first point of the line 2
    *  \param[in] line2_point2 second point of the line 2
    *  \return angle between two lines
    */
  template <class Scalar>
  inline
  Scalar lineLineAngle  ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                          const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point2
                        )
  {
    return std::acos(lineLineAngleCos<Scalar>(line1_point1, line1_point2, line2_point1, line2_point2));
  }    
  
  /** \brief Get angle between a line and a plane.
    *  \param[in] line_direction unit length direction vector of the line
    *  \param[in] plane_normal   unit normal of the plane
    */
  template <class Scalar>
  inline
  Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line_direction,
                          const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                        )
  {
    return M_PI/2 - std::acos(std::abs(utl::clampValue(line_direction.dot(plane_normal), -1.0f, 1.0f)));
  }
  
  /** \brief Get angle between a line and a plane.
    *  \param[in] line_direction     unit length direction vector of the line
    *  \param[in] plane_coefficients coefficients of the equation of the plane
    */
  template <class Scalar>
  inline
  Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line_direction,
                          const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients
                        )
  {
    Eigen::Vector3f plane_point, plane_normal;
    planeCoefficientsToPointNormal<Scalar>(plane_coefficients, plane_point, plane_normal);
    return linePlaneAngle<Scalar>(line_direction, plane_normal);
  }    
  
  /** \brief Get angle between a line and a plane.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] plane_normal unit normal of the plane
    */
  template <class Scalar>
  inline
  Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line1_point,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point,
                          const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                        )
  {
    return linePlaneAngle<Scalar>((line1_point - line2_point).normalized(), plane_normal);
  }
  
  /** \brief Get angle between a line and a plane.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] plane_coefficients coefficients of the equation of the plane
    */
  template <class Scalar>
  inline
  Scalar linePlaneAngle ( const Eigen::Matrix< Scalar, 3, 1> &line1_point,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point,
                          const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients
                        )
  {
    return linePlaneAngle<Scalar>((line1_point - line2_point).normalized(), plane_coefficients);
  }
          
  /** \brief Get counter clockwise difference between two angles (in radians)
    * \param[in] angle_start    start angle
    * \param[in] angle_end      end angle 
    * \return angular distance from start angle to end angle
    */
  template <typename Scalar>
  inline Scalar
  angleDifferenceCCW (const Scalar start_angle, const Scalar end_angle)
  {
    return utl::remainder(end_angle - start_angle, static_cast<Scalar>(2 * M_PI));
  }    
  
  //--------------------------------------------------------------------------
  // Projections
  //--------------------------------------------------------------------------
  
  /** \brief Project point on a line.
    *  \param[in] point point to be projected
    *  \param[in] line_point1  first points of a line
    *  \param[in] line_point2  second point of a line
    *  \return point projected onto a line
    */
  template <class Scalar>
  inline Eigen::Matrix< Scalar, 3, 1>
  projectPointToLine (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2)
  {
    Eigen::Matrix< Scalar, 3, 1> line_vector = line_point2 - line_point1;
    return line_point1 + (point - line_point1).dot(line_vector) * line_vector / line_vector.dot(line_vector);
  }    
  
  /** \brief Project point on a plane.
    *  \param[in] point point to be projected
    *  \param[in] plane_point  a point on the plane
    *  \param[in] plane_normal plane normal
    *  \return point projected onto a plane
    */
  template <class Scalar>
  inline Eigen::Matrix< Scalar, 3, 1>
  projectPointToPlane (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 3, 1> &plane_point, const Eigen::Matrix< Scalar, 3, 1> &plane_normal)
  {
    return point - plane_normal * pointToPlaneSignedDistance<Scalar>(point, plane_point, plane_normal);
  }
  
  /** \brief Project point on a plane.
    *  \param[in] point point to be projected
    *  \param[in] plane_coefficients coefficients of an equation of a plane
    *  \return point projected onto a plane
    */
  template <class Scalar>
  inline Eigen::Matrix< Scalar, 3, 1>
  projectPointToPlane (const Eigen::Matrix< Scalar, 3, 1> &point, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
  {
    Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
    planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);
    return projectPointToPlane<Scalar>(point, plane_point, plane_normal);
  }

  //--------------------------------------------------------------------------
  // Intersections
  //--------------------------------------------------------------------------
  
  /** \brief Compute an intersection point between a line and a plane.
    *  \param[in] line_point1 first point of the line
    *  \param[in] line_point2 second point of the line
    *  \param[in] plane plane coefficients (ax + by + cz + d = 0)
    *  \return point where line and plane intersect
    */
  template <class Scalar>
  inline Eigen::Matrix< Scalar, 3, 1>
  linePlaneIntersection ( const Eigen::Matrix< Scalar, 3, 1> &line_point1,
                          const Eigen::Matrix< Scalar, 3, 1> &line_point2,
                          const Eigen::Matrix< Scalar, 3, 1> &plane_point,
                          const Eigen::Matrix< Scalar, 3, 1> &plane_normal
                        )
  {
    Eigen::Matrix< Scalar, 3, 1> line_direction = line_point2 - line_point1;
    Scalar d = plane_normal.dot(plane_point - line_point1) / line_direction.dot(plane_normal);
    return line_point1 + d * line_direction;
  }
      
  /** \brief Compute an intersection point between a line anf a plane
    *  \param[in] line_point1 first point of the line
    *  \param[in] line_point2 second point of the line
    *  \param[in] plane_coefficients plane coefficients (ax + by + cz + d = 0)
    *  \return point where line and plane intersect
    */
  template <class Scalar>
  inline Eigen::Matrix< Scalar, 3, 1>
  linePlaneIntersection(const Eigen::Matrix< Scalar, 3, 1> &line_point1, const Eigen::Matrix< Scalar, 3, 1> &line_point2, const Eigen::Matrix< Scalar, 4, 1> &plane_coefficients)
  {
    Eigen::Matrix< Scalar, 3, 1> plane_point, plane_normal;
    planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);      
    return linePlaneIntersection(line_point1, line_point2, plane_point, plane_normal);
  }

  /** \brief Find the shortest line segment connecting two lines in 3d space.
    *  \param[in] line1_point1 first point of the line 1
    *  \param[in] line1_point2 second point of the line 1
    *  \param[in] line2_point1 first point of the line 2
    *  \param[in] line2_point2 second point of the line 2
    *  \param[out] seg_point1 first point of the shortest line segment
    *  \param[out] seg_point2 second points of the shortest line segment
    *  \param[in] eps maximum value of the determinant for the lines to be considered parallel
    *  \return true if lines are parallel
    *  \note http://geomalgorithms.com/a07-_distance.html
    */
  template <class Scalar>
  inline bool
  lineLineIntersection  ( const Eigen::Matrix< Scalar, 3, 1> &line1_point1,
                          const Eigen::Matrix< Scalar, 3, 1> &line1_point2,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point1,
                          const Eigen::Matrix< Scalar, 3, 1> &line2_point2,
                          Eigen::Matrix< Scalar, 3, 1> &seg_point1,
                          Eigen::Matrix< Scalar, 3, 1> &seg_point2,
                          Scalar eps = 1e-3
                        )
  {
    // Get direction vectors for both lines
    Eigen::Matrix< Scalar, 3, 1> l1 = line1_point2 - line1_point1;
    Eigen::Matrix< Scalar, 3, 1> l2 = line2_point2 - line2_point1;
    
    // Get temporary variables
    Scalar a = l1.dot(l1);
    Scalar b = l1.dot(l2);
    Scalar c = l2.dot(l2);
    Scalar d = l1.dot(line1_point1 - line2_point1);
    Scalar e = l2.dot(line1_point1 - line2_point1);
    
    // Calculate determinant and check that it is not too small
    Scalar D = a*c - b*b;
    
    if (D < eps)
      return true;
    
    // Calculate parameters
    Scalar alpha = (b*e - c*d) / D;
    Scalar beta  = (a*e - b*d) / D;
    
    // Get line segment points
    seg_point1 = line1_point1 + l1*alpha;
    seg_point2 = line2_point1 + l2*beta;

    return false;
  }    
  
  /** \brief Find an intersection of multiple planes. Two parameters are estimated:
  *  1) Intersection point that minimizes the point to plane distance for the input planes
  *  2) Intersection line direction that minizes th line plane angle for the input planes
  * Both are estimated using linear least squares (SVD).
  *  \param[in]  plane_coefficients  plane coefficients
  *  \param[out] point               point of the intersection line
  *  \param[out] direction           direction of the intersection line
  *  \return FALSE if the aren't enough input planes
  * \note at least 2 input planes are required.
  */
  inline
  bool planeIntersection  ( const std::vector<Eigen::Vector4f>  &plane_coefficients,
                            Eigen::Vector3f &point,
                            Eigen::Vector3f &direction
                          )
  {
    // Check input
    if (plane_coefficients.size() < 2)
    {
      std::cout << "[utl::geom::planeIntersection] a minimum of 2 input planes is required. Input only has "  << plane_coefficients.size() << " planes." << std::endl;
      std::cout << false;
    }
    
    // Construct Least Squares problems
    Eigen::MatrixXf A(plane_coefficients.size(), 3);
    Eigen::VectorXf b(plane_coefficients.size());
    
    for (size_t planeId = 0; planeId < plane_coefficients.size(); planeId++)
    {
      Eigen::Vector3f planeNormal (plane_coefficients[planeId][0], plane_coefficients[planeId][1], plane_coefficients[planeId][2]);
      float norm = planeNormal.norm();
      A.row(planeId) = planeNormal / norm;
      b[planeId]     = -plane_coefficients[planeId][3] / norm;
    }

    // Solve least squares
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
    point = svd.solve(b);
    direction = svd.matrixV().col(2);
    
    return true;
  }    
  
  /** \brief Find the area of intersection of two circles in 2D.
    *  \param[in]  circle1_c   center of the first cicle
    *  \param[in]  circle1_r   radius of the first cicle
    *  \param[in]  circle2_c   center of the second cicle
    *  \param[in]  circle2_r   radius of the second cicle
    *  \note http://mathforum.org/library/drmath/view/54785.html
    */
  template <class Scalar>
  inline
  Scalar circleCircleIntersectionArea ( const Eigen::Matrix< Scalar, 2, 1> circle1_c, const Scalar circle1_r,
                                        const Eigen::Matrix< Scalar, 2, 1> circle2_c, const Scalar circle2_r
                                      )
  {
    // Distance between two centers
    float c = pointToPointDistance<Scalar>(circle1_c, circle2_c);
    
    // Check if circles don't intersect - return 0
    if (circle1_r + circle2_r <= c)
      return static_cast<Scalar>(0.0);
    
    // If one circle contains the other - return area of the smaller circle
    float r_min = std::min(circle1_r, circle2_r);
    float r_max = std::max(circle1_r, circle2_r);
    if ((r_min + c) < r_max)
      return M_PI * r_min * r_min;
    
    // Otherwise calculate the area of the intersection lens
    Scalar cosCBA = (std::pow(circle2_r, 2) + pow(c, 2) - std::pow(circle1_r, 2)) / (2.0 * circle2_r * c);
    cosCBA = utl::clampValue<Scalar>(cosCBA, -1.0, 1.0);
    Scalar CBD = 2.0 * std::acos(cosCBA);
    
    Scalar cosCAB = (std::pow(circle1_r, 2) + pow(c, 2) - std::pow(circle2_r, 2)) / (2.0 * circle1_r * c);
    cosCAB = utl::clampValue<Scalar>(cosCAB, -1.0, 1.0);
    Scalar CAD = 2.0 * std::acos(cosCAB);
    
    Scalar area = std::pow(circle2_r, 2) * (CBD - std::sin(CBD)) + std::pow(circle1_r, 2) * (CAD - std::sin(CAD));
    area = area / 2.0;
    
    return area;
  }
  
  //--------------------------------------------------------------------------
  // Miscellaneous
  //--------------------------------------------------------------------------
  
  /** \brief Find a rotation matrix that aligns two vectors. Note that transformation matrix is not unique.
    *  \param target_normal target normal
    *  \param source_normal source normal
    *  \return 3x3 rotation matrix
    *  \note: http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    */
  template <typename Scalar>
  inline
  Eigen::Matrix<Scalar,3,3> alignVectors(const Eigen::Matrix<Scalar,1,3> source_vector, const Eigen::Matrix<Scalar,1,3> target_vector)
  {
    Eigen::Matrix<Scalar,1,3> source_normal = source_vector / source_vector.norm();
    Eigen::Matrix<Scalar,1,3> target_normal = target_vector / target_vector.norm();
    
    if (source_normal == target_normal)
      return Eigen::Matrix<Scalar,3,3>::Identity();
    
    Eigen::Matrix<Scalar,1,3> k = -target_normal.cross(source_normal);           // Unit vector representing the axis of rotation between source and target
    Scalar sinTheta = k.norm();                                                 // Rotation angle sine
    Scalar cosTheta = target_normal.dot(source_normal);                         // Rotation angle cosince
    
    Eigen::Matrix<Scalar,3,3> K;
    K <<    0 , -k(2),  k(1),
          k(2),     0, -k(0),
        -k(1),  k(0),     0;
        
    Eigen::Matrix<Scalar,3,3> R;
    R = Eigen::Matrix<Scalar,3,3>::Identity() + K + (1 - cosTheta) * K * K / sinTheta / sinTheta;
    
    return R;    
  }
}

#endif    // GEOMETRY_UTILITIES_HPP