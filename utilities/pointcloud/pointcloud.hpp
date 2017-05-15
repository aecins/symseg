// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef POINTCLOUD_UTILITIES_HPP
#define POINTCLOUD_UTILITIES_HPP

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/brute_force.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/angles.h>
#include <pcl/surface/convex_hull.h>

// Utilities
#include <graph/graph.hpp>
#include <geometry/geometry.hpp>

namespace utl
{
  /** \brief @b Downsample Downsamples a pointcloud using the @b VoxelGrid
    * filter. Unlike the original @b VoxelGrid filter can return
    *  - a map from downsampled indices to original cloud indices
    *  - indices of points of the original cloud that are closest to the
    *    corresponding downsampled points
    */    
  template <typename PointT>
  class Downsample: public pcl::VoxelGrid<PointT>
  {  
  protected:
//       using pcl::PCLBase<PointT>::setIndices;
    using pcl::VoxelGrid<PointT>::setDownsampleAllData;
    using pcl::VoxelGrid<PointT>::setSaveLeafLayout;
    using pcl::VoxelGrid<PointT>::setMinimumPointsNumberPerVoxel;
    using pcl::VoxelGrid<PointT>::setFilterFieldName;
    using pcl::VoxelGrid<PointT>::setFilterLimits;
    using pcl::VoxelGrid<PointT>::setFilterLimitsNegative;
    
  public:
          
    /** \brief Methods for downsampling pointclouds. */
    enum CloudDownsampleMethod
    { 
      AVERAGE,            /**< for each voxel downsampled point/normal is an average of points/normals belonging to the voxel. Normals are renormalized to unit length */
      NEAREST_NEIGHBOR    /**< for eaxh voxel downsampled point/normal is chosen to be the point/normal of the input cloud nearest to the voxel centroid */
    };              
    
    /** \brief Empty constructor. */
    Downsample ()  :
      downsample_method_ (AVERAGE),
      output_ (new pcl::PointCloud<PointT>),
      downsample_map_ (),
      non_nan_point_map_ (),
      nearest_indices_ ()
    {
      resetComputation();
      this->setSaveLeafLayout(true);
    }

    /** \brief Destructor. */
    ~Downsample ()
    {
    }
    
    /** \brief Provide a pointer to the input dataset
      *  \param cloud the const boost shared pointer to a PointCloud message
      */      
    virtual void
    setInputCloud(const typename pcl::PCLBase< PointT >::PointCloudConstPtr& cloud)
    {
      pcl::VoxelGrid<PointT>::setInputCloud(cloud);
      resetComputation();
    }

    /** \brief Provide a pointer to the input dataset
      *  \param cloud the const boost shared pointer to a PointCloud message
      */      
    virtual void
    setIndices (const pcl::IndicesPtr &indices)      
    {
      pcl::VoxelGrid<PointT>::setIndices(indices);
      resetComputation();
    }
    
    /** \brief Set downsampling method used.
      *  \param downsample_method downsample method
      */            
    inline void
    setDownsampleMethod (const CloudDownsampleMethod downsample_method)
    {
      downsample_method_ = downsample_method;
    }

    /** \brief Get downsampling method used.
      *  \param downsample_method downsample method
      */            
    inline CloudDownsampleMethod
    getDownsampleMethod ()  const
    {
      return downsample_method_;
    }
    
    /** \brief Set the voxel grid leaf size.
      *  \param[in] leaf_size the voxel grid leaf size
      */      
    inline void
    setLeafSize(const float leaf_size)
    {
      pcl::VoxelGrid<PointT>::setLeafSize(leaf_size, leaf_size, leaf_size);
      resetComputation();
    }
    
    /** \brief Get the voxel grid leaf size. */
    inline float
    getLeafSize ()  const   { return this->leaf_size_[0]; }

    /** \brief Get downsample map i.e. map from downsampled cloud points to
      * original cloud points.
      *  \param[out] downsample_map  downsample map
      */      
    inline void
    getDownsampleMap  (std::vector<std::vector<int> > &downsample_map)
    {
      computeDownsampleMap();
      downsample_map = downsample_map_;
    }

    /** \brief Get indices of points in the original cloud that are closest to
      * downsampled cloud points.
      *  \param[out] nearest_indices  nearest point indices
      */      
    inline void
    getNearestPointIndices  (std::vector<int> &nearest_indices)
    {
      computeNearestPointIndices();
      nearest_indices = nearest_indices_;
    }      
    
  private:
    
    /** \brief Downsampling method used. */
    CloudDownsampleMethod downsample_method_;
    
    /** \brief Downsampled pointcloud. */
    typename pcl::PointCloud<PointT>::Ptr output_;
    
    /** \brief Downsample map */
    std::vector<std::vector<int> > downsample_map_;

    /** \brief A map from the points of the donwsampled cloud mask to the indices of the NaN filtered downsampled cloud. */
    std::vector<int> non_nan_point_map_;
    
    /** \brief Indices of points closest to downsampled points */
    std::vector<int> nearest_indices_;

    /** \brief Get downsample map i.e. map from downsampled cloud points to
      * original cloud points.
      */      
    inline void
    computeDownsampleMap  ()
    {
      if (downsample_map_.empty())
      {
        // Check if the cloud was downsampled
        if (output_->empty())
        {
          std::cout << "[utl::Downsample::computeDownsampleMap] you must donwsample the cloud first." << std::endl;
          return;
        }
        
        // Generate map
        downsample_map_.resize(output_->size());
        for (size_t pointHRIt = 0; pointHRIt < this->indices_->size(); pointHRIt++)
        {
          int pointHRId = this->indices_->at(pointHRIt);
          int pointLRId = this->getCentroidIndex(this->input_->points[pointHRId]);
          if (non_nan_point_map_[pointLRId] != -1)
            downsample_map_[non_nan_point_map_[pointLRId]].push_back(pointHRId);
        }
      }
    }
    
    /** \brief Get indices of points in the original cloud that are closest to
      * downsampled cloud points.
      */      
    inline void
    computeNearestPointIndices  ()
    {
      if (nearest_indices_.empty())
      {          
        computeDownsampleMap();
        
        // Get nearest indices
        pcl::search::BruteForce<PointT> search;
        std::vector<int>            neighbors(1);
        std::vector<float>          distances(1);
        
        for (size_t pointId = 0; pointId < output_->size(); pointId++)
        {
          search.setInputCloud(this->input_, boost::make_shared<std::vector<int> >(downsample_map_[pointId]));
          search.nearestKSearch(output_->points[pointId], 1, neighbors, distances);
          nearest_indices_.push_back(neighbors[0]);
        }
      }
    }
          
    /** \brief Downsample the input pointcloud
      *  \param[out] output filtered pointcloud
      */
    virtual void
    applyFilter(pcl::PointCloud<PointT> &output)
    {
      // Downsample if we haven't already
      if (output_->empty())
        pcl::VoxelGrid<PointT>::applyFilter(*output_);
      
      // Remove point with NaN normals
      non_nan_point_map_.resize(output_->size(), -1);
      std::vector<int> non_nan_point_indices;
      int numFilteredPoints = 0;
      for (size_t pointId = 0; pointId < output_->size(); pointId++)
      {
        Eigen::Vector3f normal = output_->points[pointId].getNormalVector3fMap();
        if (pcl_isfinite(normal[0]) && pcl_isfinite(normal[1]) && pcl_isfinite(normal[2]))
        {
          non_nan_point_indices.push_back(pointId);
          non_nan_point_map_[pointId] = numFilteredPoints++;
        }
      }
      pcl::copyPointCloud<PointT>(*output_, non_nan_point_indices, *output_);
      
      if (downsample_method_ == AVERAGE)
      {
        output = *output_;
      }
      else if (downsample_method_ == NEAREST_NEIGHBOR)
      {
        computeNearestPointIndices();
        pcl::copyPointCloud<PointT>(*this->input_, nearest_indices_, output);
      }
      
      return;
    }
    
    /** \brief Reset computation flags. */
    inline void
    resetComputation ()
    {
      output_.reset(new pcl::PointCloud<PointT>);
      downsample_map_.clear();
      non_nan_point_map_.clear();
      nearest_indices_.clear();
    }
  };

  /** \brief Fit a plane to a pointcloud.
    *  \param[in] cloud input cloud
    *  \param[in] indices  indices of the points used to calculate the plane
    *  \param[out] plane_coefficients coefficients of a plane (ax + by + cz + d = 0)
    */
  template <typename PointT>
  inline
  void fitPlane ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_coefficients)
  {
    //----------------------------------------------------------------------------
    // Check that we have a sufficient number of points
    if (cloud->size() < 3)
    {
      std::cout << "[utl::fitPlane3D] input cloud contains fewer that 3 points. Can not fit a plane." << std::endl;
      abort();
    }

    //----------------------------------------------------------------------------
    // Fit plane using PCA
    pcl::PCA<PointT> pcaSolver;
    pcaSolver.setInputCloud (cloud);
    pcaSolver.setIndices (boost::make_shared<std::vector<int> > (indices));

    // Extract plane point and normal
    Eigen::Vector3f point   = pcaSolver.getMean().head(3);
    Eigen::Vector3f normal  = pcaSolver.getEigenVectors().col(2);

    // Convert to plane coefficients
    utl::pointNormalToPlaneCoefficients<float>(point, normal, plane_coefficients);
  }    
  
  /** \brief Fit a plane to a pointcloud.
    *  \param[in] cloud input cloud
    *  \param[out] plane_coefficients coefficients of a plane (ax + by + cz + d = 0)
    */
  template <typename PointT>
  inline
  void fitPlane ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud, Eigen::Vector4f &plane_coefficients)
  {
    // Create fake indices
    std::vector<int> indices (cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      indices[pointId] = pointId;
    
    // Fit plane
    fitPlane<PointT>(cloud, indices, plane_coefficients);
  }    
  
  /** \brief @b ConvexHull2D Projects pointcloud to a plane and computes the 
    * 2D convex hull of the projected points. The projection plane can be set
    * by the user. If it is not set it is computed automatically by
    * fitting a plane to the input pointcloud using PCA.
    */
  template<typename PointT>
  class ConvexHull2D : public pcl::PCLBase<PointT>
  {  
  protected:
    using pcl::PCLBase<PointT>::input_;
    using pcl::PCLBase<PointT>::indices_;
    using pcl::PCLBase<PointT>::fake_indices_;
    using pcl::PCLBase<PointT>::initCompute;
    using pcl::PCLBase<PointT>::deinitCompute;
    
  public:
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    
  public:

    /** \brief Empty constructor. */
    ConvexHull2D ()  :
      plane_coefficients_ (Eigen::Vector4f::Zero()),
      compute_plane_ (true),
      chull_ ()
    {
      chull_.setDimension(2);
      chull_.setComputeAreaVolume(true);
    }

    /** \brief Destructor. */
    virtual ~ConvexHull2D ()
    {
      input_projected_.reset();
    }
    
    /** \brief Provide a pointer to the input cloud. */
    void 
    setInputCloud (const PointCloudConstPtr &cloud)
    {
      pcl::PCLBase<PointT>::setInputCloud (cloud);
      resetComputation ();
      if (fake_indices_)
        indices_->clear();
    }
    
    /** \brief Set coefficients of the plane to which the points will be projected. */            
    inline void
    setPlaneCoefficients (const Eigen::Vector4f &plane_coefficients)
    {
      plane_coefficients_ = plane_coefficients;
      compute_plane_ = false;
    }
    
    /** \brief Get plane coefficients of the plane to which the points will be projected (either set by user or computed automatically). */
    inline Eigen::Vector4f
    getPlaneCoefficients ()  const  { return plane_coefficients_; }
    
    /** \brief Get input cloud projected on the 2D plane. Need to run reconstruct first*/
    inline PointCloudConstPtr
    getInputCloudProjected () const { return (input_projected_); }

    /** \brief Compute a convex hull for all points given.
      * \param[out] points the resultant points lying on the convex hull.
      */
    void
    reconstruct (PointCloud &points)
    {
      // Initialize computation and check that points and indices are not empty
      if (!initCompute () || input_->points.empty () || indices_->empty ())
      {
        points.points.clear ();
        return;
      }
      
      // Fit a plane to the input cloud if required
      if (compute_plane_)
      {
        utl::fitPlane<PointT>(input_, *indices_, plane_coefficients_);
      }
      else if (plane_coefficients_ == Eigen::Vector4f::Zero ())
      {
        std::cout << "[utl::ConvexHull2D::reconstruct] plane coefficients not provided but plane is not set to be computed automatically. This is not supposed to happen." << std::endl;
        return;
      }
      
      // Project points onto the plane
      input_projected_.reset(new PointCloud);
      input_projected_->resize(input_->size());
      
      for (size_t pointIdIt = 0; pointIdIt < indices_->size(); pointIdIt++)
      {
        int pointId = (*indices_)[pointIdIt];
        Eigen::Vector3f point = input_->points[pointId].getVector3fMap();
        input_projected_->points[pointId].getVector3fMap() = utl::projectPointToPlane<float>(point, plane_coefficients_);
      }
      
      // Compute convex hull
      chull_.setInputCloud (input_projected_);
      chull_.setIndices (indices_);
      chull_.reconstruct (points);
      
      // Deinit
      deinitCompute ();
    }
    
    /** \brief Returns the total area of the convex hull. */
    double
    getTotalArea () const { return (chull_.getTotalArea()); }
    
  private:

    /** \brief Coefficients of the plane to which the points are projected. */
    Eigen::Vector4f plane_coefficients_;
    
    /** \brief Flag indicating whether projection plane should be computed automatically. */
    bool compute_plane_;

    /** \brief Input cloud projected onto a 2D plane. */
    PointCloudPtr input_projected_;
    
    /** \brief Convex hull object. */
    typename pcl::ConvexHull<PointT>  chull_;
    
    /** \brief Reset intermideate computation results. */
    inline void
    resetComputation ()
    {
      input_projected_.reset();
      if (compute_plane_)
        plane_coefficients_ = Eigen::Vector4f::Zero();
    }
  };
  
  /** \brief Generate graph structure representing local connectivity between
    * points in a pointcloud. Each point is connected to its k nearest
    * neighbors.
    *  \param[in]  cloud             input cloud
    *  \param[in]  indices           indices of the points to be analyzed
    *  \param[in]  num_neighbours    maximum number of neighbours
    *  \param[out] graph             graph
    *  \note Note that a point may end up being connected to more than
    * num_neighbors points. Consider points A and B. B is within radius of A
    * but is not one of the num_neighbors closest points of A. On the other 
    * hand A is within num_neighbors closest points of B. This means that 
    * point A will be connected to num_neighbors of it's own neighbors and 
    * also to B.
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityNearestK ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                      const std::vector<int>                            &indices,
                                      utl::GraphBase<NeighborT, EdgeT>                  &graph,
                                      const int                                         num_neighbours

                                  )
  {
    // Prepare graph structure
    graph.preallocateVertices(cloud->size());
    
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud, boost::make_shared<std::vector<int> > (indices));

    // Loop over all points
    for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
    { 
      int pointId = indices[pointIdIt];
      
      // Find nearest neighbours
      std::vector<float>  distances(num_neighbours);
      std::vector<int>    neighbors(num_neighbours);
      searchTree.nearestKSearch(pointIdIt, num_neighbours, neighbors, distances);
          
      // Add corresponding edges to the graph
      for (size_t nbrId = 0; nbrId < neighbors.size(); nbrId++)
        if (pointId != neighbors[nbrId])
          graph.addEdge(pointId, neighbors[nbrId]);
    }

    // If there are no edges - return false
    if (graph.getNumEdges() < 1)
    {
      std::cout << "[utl::getCloudConnectivityNearestK] no neighbouring points were found\n";
      return false;
    }
      
    // Otherwise return true
    return true;
  }
  
  /** \brief Generate graph structure representing local connectivity between
    * points in a pointcloud. Each point is connected to its k nearest
    * neighbors.
    *  \param[in]  cloud             input cloud
    *  \param[in]  num_neighbours    maximum number of neighbours
    *  \param[out] graph             graph
    *  \return false if no edges were found, true otherwise
    *  \note Note that a point may end up being connected to more than
    * num_neighbors points. Consider points A and B. B is within radius of A
    * but is not one of the num_neighbors closest points of A. On the other 
    * hand A is within num_neighbors closest points of B. This means that 
    * point A will be connected to num_neighbors of it's own neighbors and 
    * also to B.
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityNearestK ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                      utl::GraphBase<NeighborT, EdgeT>           &graph,
                                      const int                                         num_neighbours

                                  )
  {
    // Create fake indices
    std::vector<int> fake_indices;
    fake_indices.resize(cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      fake_indices[pointId] = pointId;
      
    // Build connectivity graph
    return getCloudConnectivityNearestK<PointT>(cloud, fake_indices, graph, num_neighbours);
  }
  
  /** \brief Generate graph structure representing local connectivity between
    * points in a pointcloud. Each point is connected to it's k nearest
    * neighbors within a radius r.
    *  \param[in]  cloud             input cloud
    *  \param[in]  indices           indices of the points to be analyzed
    *  \param[in]  radius            radius within which neighbours are searched
    *  \param[out] graph             graph
    *  \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
    *  \return false if no edges were found, true otherwise
    *  \note Note that a point may end up being connected to more than
    * num_neighbors points. Consider points A and B. B is within radius of A
    * but is not one of the num_neighbors closest points of A. On the other 
    * hand A is within num_neighbors closest points of B. This means that 
    * point A will be connected to num_neighbors of it's own neighbors and 
    * also to B.
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                    const std::vector<int>                            &indices,
                                    const pcl::search::KdTree<PointT>                 &search_tree,
                                    utl::GraphBase<NeighborT, EdgeT>                  &graph,
                                    const float                                       radius,
                                    const int                                         num_neighbours = 0
                                  )
  {
    if (radius <= 0.0f)
    {
      std::cout << "[utl::getCloudConnectivityRadius] search radius must be positive." << std::endl;
      std::cout << "[utl::getCloudConnectivityRadius] input radius: " << radius << std::endl;
      return false;
    }
    
    // Prepare graph structure
    graph.preallocateVertices(cloud->size());
    
    // Loop over all points
    for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
    { 
      int pointId = indices[pointIdIt];

      // Find nearest neighbours
      std::vector<float>  distances;
      std::vector<int>    neighbors;
      search_tree.radiusSearch(pointIdIt, radius, neighbors, distances, num_neighbours);
          
      // Add corresponding edges to the graph
      for (size_t nbrId = 0; nbrId < neighbors.size(); nbrId++)
      {
        if (pointId != neighbors[nbrId])
          graph.addEdge(pointId, neighbors[nbrId]);
      }
    }
        
    // If there are no edges - return false
    if (graph.getNumEdges() < 1)
    {
      std::cout << "[utl::getCloudConnectivityRadius] no neighbouring points were found." << std::endl;
      return false;
    }
      
    // Otherwise return true
    return true;
  }
  
  /** \brief Generate graph structure representing local connectivity between
    * points in a pointcloud. Each point is connected to it's k nearest
    * neighbors within a radius r.
    *  \param[in]  cloud             input cloud
    *  \param[in]  indices           indices of the points to be analyzed
    *  \param[in]  radius            radius within which neighbours are searched
    *  \param[out] graph             graph
    *  \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
    *  \return false if no edges were found, true otherwise
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                    const std::vector<int>                            &indices,
                                    utl::GraphBase<NeighborT, EdgeT>           &graph,
                                    const float                                       radius,
                                    const int                                         num_neighbours = 0
                                  )
  {
    // Prepare search tree
    pcl::search::KdTree<PointT> searchTree;
    searchTree.setInputCloud(cloud, boost::make_shared<std::vector<int> > (indices));

    return getCloudConnectivityRadius(cloud, indices, searchTree, graph, radius, num_neighbours);
  }
  
  /** \brief Generate graph structure representing local connectivity between
    * points in the pointcloud. Each point is connected to it's k nearest
    * neighbors within a radius r.
    *  \param[in]  cloud             input cloud
    *  \param[in]  indices           indices of the points to be analyzed
    *  \param[in]  radius            radius within which neighbours are searched
    *  \param[out] graph             graph
    *  \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
    *  \return false if no edges were found, true otherwise
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                    const pcl::search::KdTree<PointT>                 &search_tree,
                                    utl::GraphBase<NeighborT, EdgeT>           &graph,
                                    const double                                      radius,                                      
                                    const int                                         num_neighbours = 0
                                  )
  {
    // Create fake indices
    std::vector<int> fake_indices;
    fake_indices.resize(cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      fake_indices[pointId] = pointId;
      
    // Build connectivity graph
    return getCloudConnectivityRadius<PointT>(cloud, fake_indices, search_tree, graph, radius, num_neighbours);
  }
  
  /** \brief Generate graph structure representing local connectivity between
    * points in the pointcloud. Each point is connected to it's k nearest
    * neighbors within a radius r.
    *  \param[in]  cloud             input cloud
    *  \param[in]  indices           indices of the points to be analyzed
    *  \param[in]  radius            radius within which neighbours are searched
    *  \param[out] graph             graph
    *  \param[in]  num_neighbours    maximum number of neighbours (if set to 0 - all neighbours will be included)
    *  \return false if no edges were found, true otherwise
    */
  template <typename PointT, typename NeighborT, typename EdgeT>
  inline
  bool getCloudConnectivityRadius ( const typename pcl::PointCloud<PointT>::ConstPtr  &cloud,
                                    utl::GraphBase<NeighborT, EdgeT>           &graph,
                                    const double                                      radius,                                      
                                    const int                                         num_neighbours = 0
                                  )
  {
    // Create fake indices
    std::vector<int> fake_indices;
    fake_indices.resize(cloud->size());
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
      fake_indices[pointId] = pointId;
      
    // Build connectivity graph
    return getCloudConnectivityRadius<PointT>(cloud, fake_indices, graph, radius, num_neighbours);
  }
  
  /** \brief Given a point in the pointcloud and it's neighbors, check if that
    * point is a boundary point.
    * The idea is similar to occlusion boundary detection provess described in 
    * "Multi-scale Feature Extraction on Point-Sampled Surfaces" by Pauly et al.:
    *  1. Project all neighboring points onto the tangent plane of the input point.
    *  2. Find the largest angle between vectors connecting the input point to projected neighbors.
    *  3. If that angle is greater than some threshold, the input point is considered to be a boundary.
    *  \param[in]  cloud     input pointcloud
    *  \param[in]  point_id  index of the input point
    *  \param[in]  neighbors indices of neighbors of the input point
    *  \param[in]  max_angle maximum angle between two consecutive neighbor points
    *  \return TRUE if input point is a boundary point
    *  \note input pointcloud must have normals
    */
  template <typename PointT>
  bool isBoundaryPoint  ( const typename pcl::PointCloud<PointT>  &cloud,
                          const int point_id,
                          const std::vector<int> &neighbours,
                          const float max_angle = pcl::deg2rad(135.0)
                        )
  {
    // If there are no neighbours it must be an occlusion
    if (neighbours.empty())
      return true;
    
    // Project neighbour points onto the tangent plane of input point
    Eigen::Vector3f planePoint  = cloud.points[point_id].getVector3fMap();
    Eigen::Vector3f planeNormal = cloud.points[point_id].getNormalVector3fMap();
    std::vector<Eigen::Vector3f> projectedNeighbours(neighbours.size());
    
    for (size_t neighbourId = 0; neighbourId < neighbours.size(); neighbourId++)
    {
      Eigen::Vector3f neighbourPoint          = cloud.points[neighbours[neighbourId]].getVector3fMap();
      Eigen::Vector3f neighbourPointProjected = utl::projectPointToPlane(neighbourPoint, planePoint, planeNormal);
      projectedNeighbours[neighbourId]        = neighbourPointProjected;
    }
    
    // Calculate signed angles between first vector and all other vectors
    Eigen::Vector3f referenceVector = projectedNeighbours[0] - planePoint;
    std::vector<float> angles (neighbours.size());
    for (size_t neighbourId = 0; neighbourId < projectedNeighbours.size(); neighbourId++)
    {
      Eigen::Vector3f currentVector = projectedNeighbours[neighbourId] - planePoint;
      float curAngle = utl::vectorVectorAngleCW(referenceVector, currentVector, planeNormal);
      angles[neighbourId] = curAngle;
    }

    // Calculate difference between consecutinve angles
    std::sort(angles.begin(), angles.end());
    std::vector<float> angleDifference(angles.size());
    for (size_t i = 1; i < angles.size(); i++)
      angleDifference[i] = utl::angleDifferenceCCW(angles[i-1], angles[i]);
    angleDifference[0] = utl::angleDifferenceCCW(angles[angles.size()-1], angles[0]);
    
    // If maximum difference is bigger than threshold mark point as boundary point
    if (*std::max_element(angleDifference.begin(), angleDifference.end()) > max_angle)
      return true;
    else
      return false;
  }

  /** \brief Find the boundary points of a pointcloud. See @utl::isBoundaryPoint
    * for algorithm details.
    *  \param[in]  cloud           input pointcloud
    *  \param[in]  search_radius   radius used to search for point neighbors
    *  \param[out] boundary_point_ids  indices of boundary points
    *  \param[in]  max_angle maximum angle between two consecutive neighbor points
    *  \return TRUE if input point is a boundary point
    *  \note input pointcloud must have normals
    */
    // NOTE: for some reason this is REEALLLLLY slow. Running this function
    // with fake indices is order of magnitude slower than running without indices
  template <typename PointT>
  bool getCloudBoundary ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const std::vector<int> &indices,
                          const float search_radius,
                          std::vector<int> &boundary_point_ids,
                          std::vector<int> &non_boundary_point_ids,
                          const float max_angle = pcl::deg2rad(135.0)
                        )
  {
    if (search_radius <= 0.0f)
    {
      std::cout << "[utl::getCloudBoundary] search radius must be positive." << std::endl;
      std::cout << "[utl::getCloudBoundary] input radius: " << search_radius << std::endl;
      return false;
    }
    
    boundary_point_ids.resize(0);
    non_boundary_point_ids.resize(0);
    
    // Prepare search tree
    typename pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud, boost::make_shared<std::vector<int> >(indices));
    
    // Loop over cloud points
    for (size_t pointIdIt = 0; pointIdIt < indices.size(); pointIdIt++)
    {
      int pointId = indices[pointIdIt];
      
      // Find point neighbors
      std::vector<float>  distancesSquared;
      std::vector<int>    neighbors;
      tree.radiusSearch(pointIdIt, search_radius, neighbors, distancesSquared);
      
      if (neighbors.size() < 1)         // If there are no neighbors - do nothing. This shouldn't really happen unless search
        continue;                       // radius is 0. In that case function will find no boundary points.
      
      std::vector<int> neighborsFirstExcluded (neighbors.begin()+1, neighbors.end());
      
      // Check if point is a boundary point
      if (utl::isBoundaryPoint<PointT>(*cloud, pointId, neighborsFirstExcluded))
        boundary_point_ids.push_back(pointId);
      else
        non_boundary_point_ids.push_back(pointId);
    }
    
    return true;
  }
  
//     /** \brief Find the boundary points of a pointcloud. See @utl::isBoundaryPoint
//      * for algorithm details.
//      *  \param[in]  cloud           input pointcloud
//      *  \param[in]  search_radius   radius used to search for point neighbors
//      *  \param[out] boundary_point_ids  indices of boundary points
//      *  \param[in]  max_angle maximum angle between two consecutive neighbor points
//      *  \return TRUE if input point is a boundary point
//      *  \note input pointcloud must have normals
//      */
//     template <typename PointT>
//     void getCloudBoundary ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
//                             const float search_radius,
//                             std::vector<int> &boundary_point_ids,
//                             const float max_angle = pcl::deg2rad(135.0)
//                           )
//     {
//       std::vector<int> fake_indices (cloud->size());
//       getCloudBoundary<PointT>(cloud, fake_indices, search_radius, boundary_point_ids, max_angle);
//     }

  /** \brief Find the boundary points of a pointcloud. See @utl::isBoundaryPoint
    * for algorithm details.
    *  \param[in]  cloud           input pointcloud
    *  \param[in]  search_radius   radius used to search for point neighbors
    *  \param[out] boundary_point_ids  indices of boundary points
    *  \param[in]  max_angle maximum angle between two consecutive neighbor points
    *  \return TRUE if input point is a boundary point
    *  \note input pointcloud must have normals
    */
  template <typename PointT>
  void getCloudBoundary ( const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const float search_radius,
                          std::vector<int> &boundary_point_ids,
                          std::vector<int> &non_boundary_point_ids,
                          const float max_angle = pcl::deg2rad(135.0)
                        )
  {
    boundary_point_ids.resize(0);
    non_boundary_point_ids.resize(0);
    
    // Prepare search tree
    typename pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(cloud);
    
    // Loop over cloud points
    for (size_t pointId = 0; pointId < cloud->size(); pointId++)
    {
      // Find point neighbors
      std::vector<float>  distancesSquared;
      std::vector<int>    neighbors;
      tree.radiusSearch(pointId, search_radius, neighbors, distancesSquared);
      
      if (neighbors.size() < 1)         // If there are no neighbors - do nothing. This shouldn't really happen unless search
        continue;                       // radius is 0. In that case function will find no boundary points.
      
      std::vector<int> neighborsFirstExcluded (neighbors.begin()+1, neighbors.end());
      
      // Check if point is a boundary point
      if (utl::isBoundaryPoint<PointT>(*cloud, pointId, neighborsFirstExcluded))
        boundary_point_ids.push_back(pointId);
      else
        non_boundary_point_ids.push_back(pointId);
    }
  }
  
  /** \brief Project a pointcloud on a plane.
    *  \param[in]  cloud_in pointcloud to be projected
    *  \param[in]  plane_point  a point on the plane
    *  \param[in]  plane_normal plane normal
    *  \param[out] cloud_out projected pointcloud
    *  \return pointcloud projected onto a plane
    */
  template <typename PointT>
  inline void
  projectCloudToPlane (const pcl::PointCloud<PointT> &cloud_in, const Eigen::Vector3f &plane_point, const Eigen::Vector3f &plane_normal, pcl::PointCloud<PointT> &cloud_out)
  {
    cloud_out = cloud_in;
    for (size_t pointId = 0; pointId < cloud_in.size(); pointId++)
      cloud_out.points[pointId].getVector3fMap() = utl::projectPointToPlane<float>(cloud_out.points[pointId].getVector3fMap(), plane_point, plane_normal);

    return;
  }

  /** \brief Project a pointcloud on a plane.
    *  \param[in]  cloud_in pointcloud to be projected
    *  \param[in]  plane_coefficients  coefficients of an equation of a plane
    *  \param[out] cloud_out projected pointcloud
    *  \return pointcloud projected onto a plane
    */
  template <typename PointT>
  inline void
  projectCloudToPlane (const pcl::PointCloud<PointT> &cloud_in, const Eigen::Vector4f &plane_coefficients, pcl::PointCloud<PointT> &cloud_out)
  {
    Eigen::Vector3f plane_point, plane_normal;
    utl::planeCoefficientsToPointNormal(plane_coefficients, plane_point, plane_normal);
    projectCloudToPlane<PointT>(cloud_in, plane_point, plane_normal, cloud_out);
    
    return;
  }
  
  /** \brief Project a pointcloud on a line.
    *  \param[in]  cloud_in pointcloud to be projected
    *  \param[in]  line_point1   first point of the line
    *  \param[in]  line_point2   second point of the line
    *  \param[out] cloud_out projected pointcloud
    *  \return pointcloud projected onto a line
    */
  template <typename PointT>
  inline void
  projectCloudToLine  (const pcl::PointCloud<PointT> &cloud_in, const Eigen::Vector3f &line_point1, const Eigen::Vector3f &line_point2, pcl::PointCloud<PointT> &cloud_out)
  {
    cloud_out = cloud_in;
    for (size_t pointId = 0; pointId < cloud_in.size(); pointId++)
      cloud_out.points[pointId].getVector3fMap() = utl::projectPointToLine<float>(cloud_out.points[pointId].getVector3fMap(), line_point1, line_point2);

    return;
  }
}

#endif  // POINTCLOUD_UTILITIES_HPP