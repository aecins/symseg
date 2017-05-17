// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REFLECTIONAL_SYMMETRY_DETECTION_CORE_HPP
#define REFLECTIONAL_SYMMETRY_DETECTION_CORE_HPP

// PCL
#include <pcl/search/kdtree.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>

// Occupancy map
#include <occupancy_map.hpp>

// Symmetry
#include <symmetry/refinement_base_functor.hpp>
#include <symmetry/reflectional_symmetry.hpp>

namespace sym
{
  //----------------------------------------------------------------------------
  // Initial symmetry generation
  //----------------------------------------------------------------------------
  
  /** \brief Generate points on a unit sphere such that they are evenly
   * distributed along the great circles associated with the meridians. The
   * angular step between the meridians and the poins on each meridian is
   * defined as 180 degrees divided by number of divisions.
   * Points are generated in three steps:
   *  1. Add a point at the pole
   *  2. Add points along the equator
   *  3. Add points by rotating equator points towards the poles
   * Note that points are returned such that no two returned points are
   * diametrical opposites.
  *  \param[in]  angular_step  angular steps along the great circle
  *  \param[out] points        points on a unit sphere.
  *  \return   points on a unit sphere
  */
  inline
  bool generateSpherePoints  (const int num_divs, std::vector<Eigen::Vector3f> &points)
  {
    points.resize(0);
    
    // Check that number of divisions is greater than 1
    if (num_divs <= 1)
    {
      std::cout << "[generateSpherePoints] Number of divisions must be greater than 1." << std::endl;
      return false;
    }
    
    // Prepare helper variables
    const float angular_step = M_PI / num_divs;
    bool needPolarPoint = (num_divs % 2 == 0);

    // Add polar point
    if (needPolarPoint)
      points.push_back(Eigen::Vector3f::UnitZ());
    
    // Add equator points
    Eigen::Matrix3f R;
    std::vector<Eigen::Vector3f> equator_points;
    equator_points.push_back(Eigen::Vector3f::UnitX());
    for (size_t stepId = 1; stepId < num_divs; stepId++)
    {
      R = Eigen::AngleAxisf(static_cast<float>(angular_step * stepId), Eigen::Vector3f::UnitZ());
      equator_points.push_back(R * equator_points[0]);
    }
    points.insert(points.end(), equator_points.begin(), equator_points.end());
    
    // Add equator points rotated towards the poles
    Eigen::Vector3f rotAxis;
    for (size_t stepId = 1; stepId < num_divs; stepId++)
    {
      // Don't duplicate the polar vertex
      if (needPolarPoint && (stepId * 2 == num_divs))
        continue;
      
      std::vector<Eigen::Vector3f> cur_nonequator_points;
      for (size_t pointId = 0; pointId < equator_points.size(); pointId++)
      {
        Eigen::Vector3f curEquatorPoint = equator_points[pointId];
        rotAxis = curEquatorPoint.cross(Eigen::Vector3f::UnitZ());
        R = Eigen::AngleAxisf(static_cast<float>(angular_step * stepId), rotAxis);
        cur_nonequator_points.push_back(R * curEquatorPoint);
      }
      points.insert(points.end(), cur_nonequator_points.begin(), cur_nonequator_points.end());
    }
    
    return true;
  }
    
  /** \brief Get the initial symmetries used for reflectional symmetry detection.
   *  \param[in]  cloud               input cloud
   *  \param[out] symmetries          reflectional symmetries
   *  \param[out] cloud_mean          mean of the pointcloud
   *  \param[in]  num_divisions       number of divisions of a sphere
   *  \param[in]  flatness_threhsold  maximum third eigenvalue for the segment to be considered flat
   *  \return FALSE if input pointcloud has less than three points (can't run PCA)
   */
  template <typename PointT>
  inline bool
  getInitialReflSymmetries ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                  std::vector<sym::ReflectionalSymmetry> &symmetries,
                                  Eigen::Vector3f &cloud_mean,
                                  int num_divisions,
                                  const float flatness_threhsold
                                )
  {
    symmetries.clear();
    
    // Check that input cloud has sufficient number of points
    if (cloud->size() < 3)
    {
      std::cout << "[sym::getInitialReflSymmetries] input cloud has less than three points. Aborting..." << std::endl;
      return false;
    }
    
    // Get the mean and the major axes of the pointcloud
    pcl::PCA<PointT> pcaSolver;
    pcaSolver.setInputCloud (cloud);
    cloud_mean = pcaSolver.getMean().head (3);
    Eigen::Matrix3f basis = pcaSolver.getEigenVectors();
    
    // Make sure that major axes form a right-handed coordinate system
    if (basis.col(0).cross(basis.col(1)).dot(basis.col(2)) < 0)
      basis.col(2) *= -1.0f;
            
  // NOTE: Possible optimization: if segment is flat - only generate symmetries
  // around it's "global" surface normal
//   bool segment_is_flat = pcaSolver.getEigenValues()[2] < params_.flatness_threshold;
//   if (segment_is_flat)
//   {
//     // Add primary symmetries
//     for (size_t symId = 0; symId < 2; symId++)
//       symmetries.push_back(sym::ReflectionalSymmetry(cloud_mean_, basis.col(symId)));      
//   }
//   
//   // Otherwise generate symmetries in full circle
//   else
//   {
      // Get symmetry normals
      std::vector<Eigen::Vector3f> spherePoints;
      generateSpherePoints(num_divisions, spherePoints);
      
      // Convert to symmetries (and rotate normals using major axes)
      for (size_t pointId = 0; pointId < spherePoints.size(); pointId++)
        symmetries.push_back(sym::ReflectionalSymmetry(cloud_mean, basis * spherePoints[pointId]));
//     }
    
    return true;
  }

  //----------------------------------------------------------------------------
  // Symmetry position refinement
  //----------------------------------------------------------------------------
  
  /** \brief Refine the position of a reflectional symmetry with respect to a 
   * pointcloud by shifting it along the symmetry plane normal. The shift
   * distance is calculated by:
   *  1. finding symmetric correspondences of the cloud
   *  2. calculating the median distance from the candidate plane to the
   *     correspondence midpoints.
   * Given a point it's symmetric correspondence is found by analyzing all of
   * it's cylindrical neighbors and selecting the one that minimizes the
   * symmetry normal error of fit.
   * Cylindrical neighbors are defined as points that lie inside a cylinder with
   * its axis formed by the target point and the symmetry normal.
   *  \param[in]  cloud               input cloud
   *  \param[in]  cloud_ds            downsampled input cloud
   *  \param[in]  symmetry            input symmetry
   *  \param[out] symmetry_refined    refined symmetry
   *  \param[out] correspondences     correspondences used for refinement
   *  \param[in]  search_cylinder_radius    radius of the search cylinder
   *  \param[in]  max_sym_normal_fit_error  maximum normal error of fit (used for correspondence rejection)
   *  \param[in]  min_sym_corresp_distance  maximum distance between two correspondences (used for correspondence rejection)
   *  \return     FALSE if input cloud is empty or there were no correspondences found
   */
  template <typename PointT>
  inline
  bool refineReflSymPosition  ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                const typename pcl::PointCloud<PointT>::ConstPtr &cloud_ds,
                                const sym::ReflectionalSymmetry &symmetry,
                                sym::ReflectionalSymmetry &symmetry_refined,
                                pcl::Correspondences &correspondences,
                                const float search_cylinder_radius = 0.01f,
                                const float max_sym_normal_fit_error = pcl::deg2rad(10.0f),
                                const float min_sym_corresp_distance = 0.02f
                              )
  {
    //--------------------------------------------------------------------------
    // Entry checks and parameters
    
    symmetry_refined = symmetry;
    
    if (cloud->size() == 0 || cloud_ds->size() == 0)
    {
      std::cout << "[sym::refineReflSymPosition] at least one of the input clouds is empty" << std::endl;
      return false;
    }
        
    //--------------------------------------------------------------------------
    // Find correspondences
    
    correspondences.clear();
    Eigen::Vector3f symmetryOrigin = symmetry_refined.getOrigin();
    Eigen::Vector3f symmetryNormal = symmetry_refined.getNormal();
    
    // Project input clouds onto the symmetry plane
    typename pcl::PointCloud<PointT>::Ptr cloudProjected (new pcl::PointCloud<PointT>);
    utl::projectCloudToPlane<PointT>(*cloud, symmetryOrigin, symmetryNormal, *cloudProjected);

    typename pcl::PointCloud<PointT>::Ptr cloudDSProjected (new pcl::PointCloud<PointT>);
    utl::projectCloudToPlane<PointT>(*cloud_ds, symmetryOrigin, symmetryNormal, *cloudDSProjected);
        
    // Create a search tree
    typename pcl::search::KdTree<PointT>  search_tree;
    search_tree.setInputCloud(cloudProjected);
        
    // Find correspondences
    for (size_t pointId = 0; pointId < cloudDSProjected->size(); pointId++)
    {
      // Get point normal
      Eigen::Vector3f srcPoint  = cloud_ds->points[pointId].getVector3fMap();
      Eigen::Vector3f srcNormal = cloud_ds->points[pointId].getNormalVector3fMap();
        
      // Find neighbours in a radius
      std::vector<float>  distancesSquared;
      std::vector<int>    neighbours;
      search_tree.radiusSearch(cloudDSProjected->points[pointId], search_cylinder_radius, neighbours, distancesSquared);
      
      // Find the best match
      int bestMatchId = -1;
      float minSymNormalFitError = std::numeric_limits< float >::max();
      for (size_t nbrIdIt = 0; nbrIdIt < neighbours.size(); nbrIdIt++)
      {
        int nbrId = neighbours[nbrIdIt];
        Eigen::Vector3f tgtPoint  = cloud->points[nbrId].getVector3fMap();
        Eigen::Vector3f tgtNormal = cloud->points[nbrId].getNormalVector3fMap();

        // NOTE: this is required for faster convergence. Distance along symmetry
        // normal works faster than point to point distnace
        // If the distance along the symmetry normal between the points of a symmetric correspondence is too small - reject
        if (std::abs(symmetry.pointSignedDistance(srcPoint) - symmetry.pointSignedDistance(tgtPoint)) < min_sym_corresp_distance)
          continue;
        
//         // NOTE: Looks like this gives no impovement in speed, neither in accuracy
//         if (utl::lineLineAngleCos((srcPoint - tgtPoint).normalized(), symmetryNormal) < tmp)
//           continue;
        
        // NOTE: it seems like choosing correspondences based on normal fit error
        // alone is sufficient and doesn't require point error
        float symNormalfitError = sym::getReflSymNormalFitError(srcNormal, tgtNormal, symmetry_refined, true);
                
        // If orientation fit error is too big - reject the correspondence
        if (symNormalfitError > max_sym_normal_fit_error)
          continue;

        // If this is the smallest error of fit so far - record it
        if (symNormalfitError <  minSymNormalFitError)
        {
          minSymNormalFitError = symNormalfitError;
          bestMatchId = nbrId;
        }           
      }
      
      if (bestMatchId != -1)
        correspondences.push_back(pcl::Correspondence(pointId, bestMatchId, minSymNormalFitError));                
    }
    
    // Correspondence rejection one to one
    pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;
    correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);
    
    // Check if there are enough correspondences
    if (correspondences.size() == 0)
    {
//       std::cout << "[sym::refineReflSymPosition] not enough correspondences" << std::endl;
      return false;
    }
    
    //------------------------------------------------------------------------
    // Update symmetry postition
        
    // Calculate median symmetry position offset
    std::vector<float> positionFitErrors (correspondences.size());
    
    for (size_t crspId = 0; crspId < correspondences.size(); crspId++)
    {
      int queryId = correspondences[crspId].index_query;
      int matchId = correspondences[crspId].index_match;
      positionFitErrors[crspId] = sym::getReflSymPositionFitError(cloud_ds->points[queryId].getVector3fMap(), cloud->points[matchId].getVector3fMap(), symmetry);
    }
    
    // Get the median offset
    float medianPositionFitError = utl::median<float>(positionFitErrors);
    
    // Update symmetry
    symmetry_refined.setOrigin(symmetryOrigin + symmetryNormal * medianPositionFitError);
    
    return true;
  }
  
  //----------------------------------------------------------------------------
  // Symmetry global refinement
  //----------------------------------------------------------------------------
  
  /** \brief A functor for refining a symmetry candidate in a
   * Levenberg-Marquardt optimization. Given a set of symmetric correspondences
   * between oriented points and an initial reflectional symmetry candidate,
   * find a relfectional symmetry candidate that minimizes the point to plane 
   * distance between the first point corresponding point and the reflection
   * of the second corresponding point.
   * Symmetry parameters are stored in a 6 dimensional vector
   * (symmetry origin, symmetry normal).
   * \note the final refined symmetry vector may have a non-unit normal.
   */
  template <typename PointT>
  struct ReflSymRefineFunctor : BaseFunctor<float>
  {
    /** \brief Empty constructor */
    ReflSymRefineFunctor ()  {};
    
    /** \brief Compute fitness for each input point.
     *  \param[in]  x coefficients of the symmetry plane
     *  \param[out] fvec error vector
     */
    int operator()(const Eigen::VectorXf &x, Eigen::VectorXf &fvec) const
    {
      // Get current rotational symmetry 
      ReflectionalSymmetry symmetry (x.head(3), x.tail(3));

//       // Compute occlusion scores
//       for(size_t i = 0; i < this->cloud_ds_->size(); i++)
//       {
//         Eigen::Vector3f srcPoint  = cloud_ds_->points[i].getVector3fMap();
//         Eigen::Vector3f srcPointReflected   = symmetry.reflectPoint(srcPoint);
//                 
//         fvec(i) = 0.5 * std::max(0.0f, occupancy_->getNearestObstacleDistance(srcPointReflected) - 0.01f);
//       }
//       
//       // Compute symmetry scores
//       for(size_t i = 0; i < this->correspondences_.size(); i++)
//       {
//         int srcPointIndex = correspondences_[i].index_query;
//         int tgtPointIndex = correspondences_[i].index_match;
//         
//         Eigen::Vector3f srcPoint  = cloud_ds_->points[srcPointIndex].getVector3fMap();
//         Eigen::Vector3f srcNormal = cloud_ds_->points[srcPointIndex].getNormalVector3fMap();
//         Eigen::Vector3f tgtPoint  = cloud_->points[tgtPointIndex].getVector3fMap();
//         Eigen::Vector3f tgtNormal = cloud_->points[tgtPointIndex].getNormalVector3fMap();
//                 
//         // Reflect target point and normal
//         Eigen::Vector3f tgtPointReflected   = symmetry.reflectPoint(tgtPoint);
//         Eigen::Vector3f tgtNormalReflected  = symmetry.reflectNormal(tgtNormal);
//                 
//         fvec(srcPointIndex) += std::abs(utl::pointToPlaneSignedDistance<float>(srcPoint, tgtPointReflected, tgtNormalReflected));
//       }
      
      // Compute fitness
      for(size_t i = 0; i < this->correspondences_.size(); i++)
      {
        int srcPointIndex = correspondences_[i].index_query;
        int tgtPointIndex = correspondences_[i].index_match;
        
        Eigen::Vector3f srcPoint  = cloud_ds_->points[srcPointIndex].getVector3fMap();
        Eigen::Vector3f srcNormal = cloud_ds_->points[srcPointIndex].getNormalVector3fMap();
        Eigen::Vector3f tgtPoint  = cloud_->points[tgtPointIndex].getVector3fMap();
        Eigen::Vector3f tgtNormal = cloud_->points[tgtPointIndex].getNormalVector3fMap();
                
        // Reflect target point and normal
        Eigen::Vector3f tgtPointReflected   = symmetry.reflectPoint(tgtPoint);
        Eigen::Vector3f tgtNormalReflected  = symmetry.reflectNormal(tgtNormal);
        
        fvec(i) = std::abs(utl::pointToPlaneSignedDistance<float>(srcPoint, tgtPointReflected, tgtNormalReflected));
        
        // NOTE: why not use the symmetry fitness error here? I.e. the angular difference between the reflected normals?
        // It seems like the point to plane distance works better, but need more checks
//         fvec(i) = 1.0f - srcNormal.dot(tgtNormalReflected);
      }
      
      return 0;
    }
    
    /** \brief Input cloud. */
    typename pcl::PointCloud<PointT>::ConstPtr cloud_;
    
    /** \brief Downsampled input cloud. */
    typename pcl::PointCloud<PointT>::ConstPtr cloud_ds_;
    
    /** \brief Scene occupancy. */
    OccupancyMapConstPtr occupancy_;
    
    /** \brief Input correspondences. */
    pcl::Correspondences correspondences_;
        
    /** \brief Dimensionality of the optimization parameter vector. */
    int inputs() const { return 6; }
    
    /** \brief Number of points. */
//     int values() const { return this->cloud_ds_->size(); }
    int values() const { return this->correspondences_.size(); }
  };
  
  template <typename PointT>
  struct ReflSymRefineFunctorDiff : Eigen::NumericalDiff<ReflSymRefineFunctor<PointT> > {};
  
  /** \brief Given a pointcloud with normals and an initial reflectional
   * symmetry candidate, refine the reflectional symmetry such that the cloud
   * "reflects" onto itself. This is done in an ICP-like optimization scheme,
   * that alternates between symmetric correspondence estimation and symmetry
   * plane optimization:
   * 1. Reflect input pointcloud using current symmetry.
   * 2. For each point in the original cloud find the closest point of the
   *    reflected cloud.
   * 3. Refine symmetry orientation given the correspondences.
   * 4. Repeat until convergence.
   *  \param[in]  cloud               input cloud
   *  \param[in]  cloud_mean          mean point of the pointcloud
   *  \param[in]  symmetry            input symmetry
   *  \param[out] symmetry_refined    refined symmetry
   *  \param[out] correspondences           correspondences used for optimization
   *  \param[in]  max_iterations            maximum number of optimization iterations
   *  \param[in]  max_sym_normal_fit_error  maximum normal error of fit (used for correspondence rejection)
   *  \param[in]  min_sym_corresp_distance  maximum distance between two correspondences (used for correspondence rejection)
   *  \param[in]  max_sym_corresp_reflected_distance  maximum distance between the first point of a symmetric correspondence and a reflection of the second point (used for correspondence rejection)
   *  \return     FALSE if input cloud is empty or there were no correspondences found during any iteration
   */
  template <typename PointT>
  inline
  bool refineReflSymGlobal  ( const typename pcl::search::KdTree<PointT> &search_tree,
                              const typename pcl::PointCloud<PointT>::ConstPtr &cloud_ds,
                              const Eigen::Vector3f &cloud_mean,
                              const OccupancyMapConstPtr &occupancy_map,
                              const sym::ReflectionalSymmetry &symmetry,
                              sym::ReflectionalSymmetry &symmetry_refined,
                              pcl::Correspondences &correspondences,
                              const int max_iterations = 20,
                              const float max_sym_normal_fit_error = pcl::deg2rad(45.0f),
                              const float min_sym_corresp_distance = 0.02f,
                              const float max_sym_corresp_reflected_distance = 0.005f
                            )
  {    
    //--------------------------------------------------------------------------
    // Entry checks and parameters
    
    typename pcl::PointCloud<PointT>::ConstPtr cloud (new pcl::PointCloud<PointT>);
    cloud = search_tree.getInputCloud();
    
    symmetry_refined = symmetry;
    
    if (cloud->size() == 0 || cloud_ds->size() == 0)
      return false;
            
    // Correspondence rejection
    pcl::registration::CorrespondenceRejectorOneToOne correspRejectOneToOne;
    
    // Functor object
    sym::ReflSymRefineFunctorDiff<PointT> functor;
    functor.cloud_      = cloud;      
    functor.cloud_ds_   = cloud_ds;
    functor.occupancy_  = occupancy_map;
    
    //--------------------------------------------------------------------------
    // Main loop
    
    int nrIterations = 0;
    sym::ReflectionalSymmetry symmetry_prev;
    
    bool done = false;
    while (!done)
    {
      symmetry_prev = symmetry_refined;
      
      //------------------------------------------------------------------------
      // Correspondence estimation and rejection
            
      // Reset correspondences
      correspondences.clear();
      
      // Get symmetry parameters
      Eigen::Vector3f symmetryOrigin = symmetry_refined.getOrigin();
      Eigen::Vector3f symmetryNormal = symmetry_refined.getNormal();
            
      // Find correspondences
      for (size_t pointId = 0; pointId < cloud_ds->size(); pointId++)
      {
        // Get point normal
        Eigen::Vector3f srcPoint  = cloud_ds->points[pointId].getVector3fMap();
        Eigen::Vector3f srcNormal = cloud_ds->points[pointId].getNormalVector3fMap();
        
        // NOTE: somehow this gives sliiightly worse results than using the rejection below
//         // If point is too close to the symmetry plane - don't use it as a correspondence
//         if (std::abs(symmetry.pointSignedDistance(srcPoint)) < 0.01f)
//           continue;
        
        // Reflect point and normal
        Eigen::Vector3f srcPointReflected   = symmetry_refined.reflectPoint(srcPoint);
        Eigen::Vector3f srcNormalRefleclted = symmetry_refined.reflectNormal(srcNormal);
                
        // Find nearest neighbor
        std::vector<float>  distancesSquared(1);
        std::vector<int>    neighbours(1);
        PointT searchPoint;
        searchPoint.getVector3fMap() = srcPointReflected;
        searchPoint.getNormalVector3fMap() = srcNormalRefleclted;        
        search_tree.nearestKSearch (searchPoint, 1, neighbours, distancesSquared);
        
        Eigen::Vector3f tgtPoint  = cloud->points[neighbours[0]].getVector3fMap();
        Eigen::Vector3f tgtNormal = cloud->points[neighbours[0]].getNormalVector3fMap();        
                        
        // NOTE: this is required for faster convergence. Distance along symmetry
        // normal works faster than point to point distnace
        // If the distance along the symmetry normal between the points of a symmetric correspondence is too small - reject
        if (std::abs(symmetry.pointSignedDistance(srcPoint) - symmetry.pointSignedDistance(tgtPoint)) < min_sym_corresp_distance)
          continue;
        
        // If the distance between the reflected source point and it's nearest neighbor is too big - reject
        if (distancesSquared[0] > max_sym_corresp_reflected_distance * max_sym_corresp_reflected_distance)
          continue;        
        
        // NOTE: it seems like this is required for correct convergence
        // Reject correspondence if normal error is too high
        float error = sym::getReflSymNormalFitError(srcNormal, tgtNormal, symmetry_refined, true);                
        if (error > max_sym_normal_fit_error)
          continue;
        
        // If all checks passed - add correspondence
        correspondences.push_back(pcl::Correspondence(pointId, neighbours[0], distancesSquared[0]));
      }
      
      // Correspondence rejection one to one
      correspRejectOneToOne.getRemainingCorrespondences(correspondences, correspondences);
      
      // Check if there are enough correspondences
      if (correspondences.size() == 0)
        return false;
      
      //------------------------------------------------------------------------
      // Optimization
      
      // Construct optimization vector
      Eigen::VectorXf x (6);
      x.head(3) = symmetryOrigin;
      x.tail(3) = symmetryNormal;
            
      // Construct functor object
      functor.correspondences_ = correspondences;
      
      // Optimize!
      Eigen::LevenbergMarquardt<sym::ReflSymRefineFunctorDiff<PointT>, float> lm(functor);      
      lm.minimize(x);
      
      // Convert to symmetry
      symmetry_refined = sym::ReflectionalSymmetry (x.head(3), x.tail(3));
      symmetry_refined.setOriginProjected(cloud_mean);
      
      //------------------------------------------------------------------------
      // Check convergence
      
      // Check iterations
      if (++nrIterations >= max_iterations)
        done = true;
      
      // Check if symmetry has changed enough
      float angleDiff, distanceDiff;
      symmetry_refined.reflSymDifference(symmetry_prev, cloud_mean, angleDiff, distanceDiff);
      if (angleDiff < pcl::deg2rad(0.05f) && distanceDiff < 0.0001f)
        done = true;
    }
    
    return true;
  }
}

#endif    // REFLECTIONAL_SYMMETRY_DETECTION_CORE_HPP