// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef REGION_GROWING_H
#define REGION_GROWING_H

// PCL includes
#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

// Utilities includes
#include <std_vector.hpp>
#include <geometry/math.hpp>

namespace utl
{
  typedef std::vector<pcl::PointIndices> IndicesClusters;
  typedef boost::shared_ptr<std::vector<pcl::PointIndices> > IndicesClustersPtr;

  /** \brief @b RegionGrowing Performs region growing segmentation on a
   * pointcloud using a user-defined condition function.
   * 
   * Given a point in the cloud it's neighboring points are found. For all of 
   * them a two conditions are verified:
   *  1) unary condition determining if a neighboring point is a valid candidate
   *  2) binary condition between the original point and a neighboring point 
   *  determining if they make a valid pair
   * 
   * If the fraction of neighbors satisfying the unary constraint is larger than
   * a threshold and the fraction of neighbors satisfying the binary constraint 
   * is larger than (a different) threshold neighboring points satisfying both 
   * conditions are added to the cluster. The same procedure is run on the newly
   * added points. Once this process converges it is run again on the remaining points in the 
   * cloud (i.e. points that were not yet assigned to any cluster).
   */
  template<typename PointT>
  class RegionGrowing : public pcl::PCLBase<PointT>
  {
    protected:
      typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;
      typedef boost::function<bool (const PointT&)>                       unaryFunction;
      typedef boost::function<bool (const PointT&, const PointT&, float)> binaryFunction;

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
      /** \brief Constructor */
      RegionGrowing ();
      
      /** \brief Constructor */
      ~RegionGrowing ();
      
      /** \brief Provide a pointer to the input cloud. */
      void
      setInputCloud (const PointCloudConstPtr &cloud);

      /** \brief Set condition function that needs to be satisfied for a point to belong to a cluster */
      inline void
      setUnaryConditionFunction (bool (*unary_condition_function) (const PointT&));

      /** \brief Set condition function that needs to be satisfied for a point to belong to a cluster */
      inline void
      setUnaryConditionFunction (unaryFunction unary_condition_function);
      
      /** \brief Set condition function that needs to be satisfied for two neighbouring points to be in the same cluster */
      inline void
      setBinaryConditionFunction (bool (*binary_condition_function) (const PointT&, const PointT&, float));      

      /** \brief Set condition function that needs to be satisfied for two neighbouring points to be in the same cluster */
      inline void
      setBinaryConditionFunction (binaryFunction binary_condition_function);
      
      /** \brief Set the search radius for finding point neighors. */
      inline void
      setSearchRadius (float search_radius);

      /** \brief Set the search radius for expanding the segments. */
      inline float
      getSearchRadius () const;

      /** \brief Set maximum number of neighbors for each point. */
      inline void
      setNumberOfNeighbors (int num_neighbors);

      /** \brief Set the search radius for expanding the segments. */
      inline int
      getNumberOfNeighbors () const;

      /** \brief Set minimum fraction of neighbors required to satisfy unary constraint to grow the cluster. */
      inline void
      setMinValidUnaryNeighborsFraction (float valid_unary_neighbor_fraction);

      /** \brief Get minimum fraction of neighbors required to satisfy unary constraint to grow the cluster. */
      inline float
      getMinValidUnaryNeighborsFraction () const;
            
      /** \brief Set fraction of neighbors required to satisfy the condition to grow the cluster. */
      inline void
      setMinValidBinaryNeighborsFraction (float valid_binary_neighbor_fraction);

      /** \brief Get minimum fraction of neighbors required to satisfy unary constraint to grow the cluster. */
      inline float
      getMinValidBinaryNeighborsFraction () const;

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
      inline void
      setMinSegmentSize (const int min_cluster_size);
      
      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      inline int
      getMinSegmentSize ()  const;

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      inline void
      setMaxSegmentSize (const int max_cluster_size);      

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      inline int
      getMaxSegmentSize () const;
            
      /** \brief Segment the pointcloud.
       * \param[out] segments  a vector of segments where each segment is represented by the indices of points belonging to it
       */
      void
      segment (std::vector<std::vector<int> > &clusters);

    protected:
      
      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
       * the current settings. If it is possible then it returns true.
       */
      virtual bool
      prepareForSegmentation ();
      
    private:
      /** \brief A pointer to the spatial search object */
      SearcherPtr searcher_;

      /** \brief Search radius for finding point neighbors (default = -1.0) */
      float search_radius_;
      
      /** \brief Number of nearest neighbors to analyzed (default = all) */
      int num_neighbors_;
            
      /** \brief The condition function for a point that needs to be satisfied for a point to belong to a cluster */
      unaryFunction unary_condition_function_;
      
      /** \brief The condition function between two points that needs to to be satisfied for two neighboring points to belong to the same cluster.  */
      binaryFunction binary_condition_function_;

      /** \brief Minimum fraction of neighboring points that have to satisfy unary constraint to add neighbors to cluster. */
      float min_valid_unary_neighbor_fraction_;

      /** \brief Minimum fraction of neighboring points that have to satisfy binary constraint to add neighbors to cluster. */
      float min_valid_binary_neighbor_fraction_;
      
      /** \brief The minimum cluster size (default = 1) */
      int min_cluster_size_;

      /** \brief The maximum cluster size (default = unlimited) */
      int max_cluster_size_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif  // REGION_GROWING_H