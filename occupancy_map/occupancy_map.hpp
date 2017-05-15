// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef OCCUPANCY_MAP_HPP
#define OCCUPANCY_MAP_HPP

#include <mutex>
#include <omp.h>

// PCL includes
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// Octomap includes
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Utilities
#include "geometry/geometry.hpp"

class OccupancyMap
{
public:
  
  /** \brief Empty constructor. */
  OccupancyMap()
    : occupancyTree_ (0.0)
    , distanceMap_ (NULL)
    , obstacleMap_ (NULL)
  { }
  
  /** \brief Destructor. */
  ~OccupancyMap()
  {
    if (distanceMap_)
      delete distanceMap_;
  }
  
  /** \brief Read an occupancy tree from a file. Note that file is assumed to be
   * in binary format.
   *  \param[in]  filename    occupancy tree filename
   *  \return TRUE if occupancy tree was read successfully
   */ 
  bool readOccupancyTree (const std::string &filename);

  /** \brief Get the depth of the occupancy tree.
   *  \return depth of the occupancy tree
   */ 
  unsigned int getOccupancyTreeDepth () const;
  
  /** \brief Get the resolution of the occupancy tree.
   *  \return resolution of the occupancy tree
   */ 
  float getOccupancyTreeResolution () const;
  
  /** \brief Construct a distance map from an occupancy map. Occluded and
   * occupied  voxels are considered to belong to the obstacle space. Distance
   * map is constructed by recording distances to the nearest obstacle voxel.
   * Optionally obstacle space can be inflated by one voxel along the boundary 
   * between occluded and free space.
   *  \param[in]  cloud                   input pointcloud
   *  \param[in]  bbx_inflation_radius    distance map bounding box inflation radius
   *  \param[in]  depth                   depth at which the distance map is constructed
   *  \param[in]  max_distance            maximum distance in the distance map
   *  \param[in]  inflate_obstacle_space      inflate occupied space by one voxel at the boundary between occluded 
   *  \return TRUE if distance map was constructed successfully
   */
  bool distanceMapFromOccupancy (const Eigen::Vector3f &bbx_min, const Eigen::Vector3f &bbx_max, const uint16_t depth, const float max_distance, const bool inflate_obstacle_space);
    
  /** \brief Set scene bounding planes
   *  \param[in]  bounding_planes  a vector of coefficients of the bounding planes of the scene
   *  \param[in]  offsets           an offset of the plane along the normal
   *  \note: the normal of the bounding planes matters
   */ 
  bool setBoundingPlanes  (const std::vector<Eigen::Vector4f> &bounding_planes, const std::vector<float> &offsets = std::vector<float>(0));
  
  /** \brief Get the minimum signed distance between a point and the bounding
   * planes.
   *  \param[in]  point   point
   *  \return   minimum signed distance between the point and the bounding planes
   */
  float getPointBoundingPlaneMinSignedDistance (const Eigen::Vector3f &point) const;

  /** \brief Get the distance to the nearest occluded/occupied cell
   *  \param[in]  point   point
   *  \return   distance to the nearest occluded/occupied cell
   */
  float getNearestOccludedOccupiedDistance (const Eigen::Vector3f &point) const;
  
  /** \brief Get the euclidean distance of a point to the nearest obstacle.
   *  \param[in]  point   point
   *  \return Euclidean distance to the closest obstacle.
   */
  float getNearestObstacleDistance (const Eigen::Vector3f &point) const;

  /** \brief Get the euclidean distance of the point to the nearest obstacle.
   *  \param[in]  point   point
   *  \return Coordinates of the nearest obstacle
   */
  Eigen::Vector3f getNearestObstacle (const Eigen::Vector3f &point) const;
     
  /** \brief Check if a point is occluded or not.
   *  \param[in]  point   point
   *  \return TRUE if point falls into occluded space, FALSE otherwise.
   */
  bool isPointOccluded (const Eigen::Vector3f &point) const;
  
  /** \brief Visualize the outer layer of the occluded space.
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  id          point cloud object id prefix (default: occluded_space)
   */
  void showOccludedSpaceBoundaryMesh  ( pcl::visualization::PCLVisualizer &visualizer,
                                        const std::string &id = "occluded_space_boundary"
                                      ) const;

  /** \brief Visualize the boundary between the obstacle space and the free space.
   *  \param[in]  visualizer  visualizer object
   *  \param[in]  id          point cloud object id prefix (default: occluded_space)
   */
  void showObstacleSpaceBoundaryMesh  ( pcl::visualization::PCLVisualizer &visualizer,
                                        const std::string &id = "obstacle_space_boundary"
                                      ) const;
                                      
private:
  /** \brief Convert an occupancy tree key to a distance map key.
   *  \param[in]  key_oct   occupancy tree key
   *  \return corresponding distance map key
   */
  octomap::OcTreeKey occupancyTreeKeyToDistanceMapKey (const octomap::OcTreeKey &key_oct) const;
  
  /** \brief Convert an distance map key to an occupancy tree key.
   *  \param[in]  key_oct   occupancy tree key
   *  \return corresponding distance map key
   */
  octomap::OcTreeKey distanceMapKeyToOccupancyTreeKey (const octomap::OcTreeKey &key_dm) const;
  
  /** \brief Get an occupancy tree key corresponding to a point in 3D space.
   *  \param[in]  point       point
   *  \return occupancy tree key corresponding to the input 3D point.
   */
  octomap::OcTreeKey getOccupanyTreeKey (const Eigen::Vector3f &point) const;

  /** \brief Get a distance map key corresponding to a point in 3D space.
   *  \param[in]  point       point
   *  \return distance map key corresponding to the input 3D point.
   */
  octomap::OcTreeKey getDistanceMapKey (const Eigen::Vector3f &point) const;
  
  /** \brief Construct a polygon mesh corresponding to the boundary of the 
   * occluded space.
   *  \param[out] vertices   mesh vertices
   *  \param[out] polygons   mesh polygons
   *  \return FALSE if either occupancy tree or distance map doesn't exist.
   */  
  bool getOccludedSpaceBoundaryMesh  (pcl::PointCloud< pcl::PointXYZ >& vertices, std::vector< pcl::Vertices >& polygons) const;
  
  /** \brief Construct a polygon mesh corresponding to the boundary of the 
   * obstacle space used in the distance map.
   *  \param[out] vertices   mesh vertices
   *  \param[out] polygons   mesh polygons
   *  \return FALSE if either occupancy tree or distance map doesn't exist.
   */
  bool getObstacleSpaceBoundaryMesh  (pcl::PointCloud<pcl::PointXYZ> &vertices, std::vector<pcl::Vertices> &polygons) const;

//   bool getObstacleSpaceBoundaryMeshTest  (pcl::PointCloud<pcl::PointXYZ> &vertices, std::vector<pcl::Vertices> &polygons) const;    
  
  /** \brief Create a mesh representing a boundary between neighboring voxels.
   *  \param[in]  voxel_neighbor_list   list of voxel centers and their neigbors
   *  \param[out] vertices   mesh vertices
   *  \param[out] cloud   mesh polygons
   */
  void generateVoxelBoundaryMesh  ( const std::vector<std::pair<octomap::point3d, int> > voxel_neighbor_list,
                                    pcl::PointCloud<pcl::PointXYZ> &vertices,
                                    std::vector<pcl::Vertices> &polygons
                                  ) const;
                                  
  /** \brief Neighborhood used to find adjacent voxels. */
  static std::vector<std::vector<int> > createVoxelNeighborhood26 ();
  static std::vector<std::vector<int> > createVoxelNeighborhood18 ();
  static std::vector<std::vector<int> > createVoxelNeighborhood6 ();
                                  
  /** \brief Occupancy tree. Stores scene occupancy information. */
  octomap::OcTree occupancyTree_;
  
  /** \brief Distance map. Stores distance to the nearest surface for all scene voxels. */
  DynamicEDT3D* distanceMap_;
  
  /** \brief Maximum distance in the distance map. */
  float distanceMapMaxDist_;
  
  /** \brief A 3D boolean array storing the locations of obstacle voxels in the scene. */
  bool*** obstacleMap_;
  
  /** \brief Depth of the occupancy map. */
  uint16_t depth_;
  
  /** \brief Bounding box minimum point. */
  octomap::point3d bbxMin_;
  
  /** \brief Bounding box maximum point. */
  octomap::point3d bbxMax_;
  
  /** \brief Size of the distance map voxel expressed in occupancy tree voxel size. */
  uint16_t dmVoxelSize_;
  
  /** \brief Origin of the distance map expressed in the occupancy tree key coordinates. */
  octomap::OcTreeKey octToDmOffset_;
  
  /** \brief Bounding planes of the scene. */
  std::vector<Eigen::Vector4f> boundingPlanes_;
};

////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<int> > OccupancyMap::createVoxelNeighborhood26 ()
{
  std::vector<std::vector<int> > voxelNeighborhood26 (26);
  voxelNeighborhood26[0]   = { 1,  0,  0};
  voxelNeighborhood26[1]   = {-1,  0,  0};
  voxelNeighborhood26[2]   = { 0,  1,  0};
  voxelNeighborhood26[3]   = { 0, -1,  0};
  voxelNeighborhood26[4]   = { 0,  0,  1};
  voxelNeighborhood26[5]   = { 0,  0, -1};

  voxelNeighborhood26[6]   = { 1,  1,  0};
  voxelNeighborhood26[7]   = { 1,  0,  1};
  voxelNeighborhood26[8]   = { 0,  1,  1};
  voxelNeighborhood26[9]   = { 1, -1,  0};
  voxelNeighborhood26[10]  = { 1,  0, -1};
  voxelNeighborhood26[11]  = { 0,  1, -1};
  voxelNeighborhood26[12]  = {-1,  1,  0};
  voxelNeighborhood26[13]  = {-1,  0,  1};
  voxelNeighborhood26[14]  = { 0, -1,  1};
  voxelNeighborhood26[15]  = {-1, -1,  0};
  voxelNeighborhood26[16]  = {-1,  0,  1};
  voxelNeighborhood26[17]  = { 0, -1, -1};
  voxelNeighborhood26[15]  = { 1, -1,  0};
  voxelNeighborhood26[16]  = {-1,  0,  1};
  voxelNeighborhood26[17]  = { 0, -1, -1};

  voxelNeighborhood26[18]  = { 1,  1,  1};
  voxelNeighborhood26[19]  = {-1,  1,  1};
  voxelNeighborhood26[20]  = { 1, -1,  1};
  voxelNeighborhood26[21]  = { 1,  1, -1};
  voxelNeighborhood26[22]  = {-1, -1,  1};
  voxelNeighborhood26[23]  = {-1,  1, -1};
  voxelNeighborhood26[24]  = { 1, -1, -1};
  voxelNeighborhood26[25]  = {-1, -1, -1};
  
  return voxelNeighborhood26;
}

std::vector<std::vector<int> > OccupancyMap::createVoxelNeighborhood6 ()
{
  std::vector<std::vector<int> > voxelNeighborhood = createVoxelNeighborhood26();
  return std::vector<std::vector<int> > (voxelNeighborhood.begin(), voxelNeighborhood.begin()+6);
}

std::vector<std::vector<int> > OccupancyMap::createVoxelNeighborhood18 ()
{
  std::vector<std::vector<int> > voxelNeighborhood = createVoxelNeighborhood26();
  return std::vector<std::vector<int> > (voxelNeighborhood.begin(), voxelNeighborhood.begin()+18);
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::readOccupancyTree(const std::string& filename)
{
  // Delete distance map if it already exists
  if (distanceMap_)
  {
    distanceMap_->~DynamicEDT3D();
    delete distanceMap_;
    distanceMap_ = NULL;
  }
  
  // Read occupancy tree
  if (!occupancyTree_.readBinary(filename))
  {
    std::cout << "[OccupancyMap::readOccupancyTree] Could not read octomap file '" + filename + "'" << std::endl;
    return false;
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
unsigned int OccupancyMap::getOccupancyTreeDepth() const
{
  return occupancyTree_.getTreeDepth();
}

////////////////////////////////////////////////////////////////////////////////
float OccupancyMap::getOccupancyTreeResolution() const
{
  return occupancyTree_.getResolution();
}

////////////////////////////////////////////////////////////////////////////////
octomap::OcTreeKey OccupancyMap::occupancyTreeKeyToDistanceMapKey (const octomap::OcTreeKey &key_oct) const
{
  octomap::OcTreeKey key_dm;
  key_dm[0] = (key_oct[0] - octToDmOffset_[0]) / dmVoxelSize_;
  key_dm[1] = (key_oct[1] - octToDmOffset_[1]) / dmVoxelSize_;
  key_dm[2] = (key_oct[2] - octToDmOffset_[2]) / dmVoxelSize_;
  return key_dm;    
}

////////////////////////////////////////////////////////////////////////////////
octomap::OcTreeKey OccupancyMap::distanceMapKeyToOccupancyTreeKey (const octomap::OcTreeKey &key_dm) const
{
  octomap::OcTreeKey key_oct;    
  key_oct[0] = key_dm[0] * dmVoxelSize_ + octToDmOffset_[0];
  key_oct[1] = key_dm[1] * dmVoxelSize_ + octToDmOffset_[1];
  key_oct[2] = key_dm[2] * dmVoxelSize_ + octToDmOffset_[2];
  return key_oct;
}

////////////////////////////////////////////////////////////////////////////////
octomap::OcTreeKey OccupancyMap::getOccupanyTreeKey (const Eigen::Vector3f &point) const
{
  return occupancyTree_.coordToKey(octomap::point3d (point[0], point[1], point[2]), depth_);
}

////////////////////////////////////////////////////////////////////////////////
octomap::OcTreeKey OccupancyMap::getDistanceMapKey  (const Eigen::Vector3f &point) const
{
  octomap::OcTreeKey key_oct = getOccupanyTreeKey(point);
  return occupancyTreeKeyToDistanceMapKey(key_oct);
}

////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::generateVoxelBoundaryMesh(const std::vector<std::pair<octomap::point3d, int> > voxel_neighbor_list, pcl::PointCloud< pcl::PointXYZ >& vertices, std::vector< pcl::Vertices >& polygons) const
{
  vertices.resize(voxel_neighbor_list.size() * 4);
  polygons.resize(voxel_neighbor_list.size() * 2);
  
  float l = occupancyTree_.getNodeSize(depth_) / 2.0f;
  
  #pragma omp parallel for
  for (size_t vxlId = 0; vxlId < voxel_neighbor_list.size(); vxlId++)
  {
    octomap::point3d pointOct = voxel_neighbor_list[vxlId].first;
    int nbrId = voxel_neighbor_list[vxlId].second;
    pcl::PointXYZ p1, p2, p3, p4;
    
    switch (nbrId)
    {
      case 0:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() + l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() + l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() - l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() - l);                    
        break;
      }
      
      case 1:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() + l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() + l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() - l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() - l);                    
        break;
      }
        
      case 2:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() + l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() + l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() - l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() - l);
        break;
      }
      
      case 3:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() + l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() + l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() - l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() - l);
        break;
      }
      
      case 4:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() + l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() + l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() + l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() + l);
        break;
      }

      case 5:
      {
        p1.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() + l, pointOct.z() - l);
        p2.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() + l, pointOct.z() - l);
        p3.getVector3fMap() = Eigen::Vector3f(pointOct.x() - l, pointOct.y() - l, pointOct.z() - l);
        p4.getVector3fMap() = Eigen::Vector3f(pointOct.x() + l, pointOct.y() - l, pointOct.z() - l);
        break;
      }
    }
    
    vertices[vxlId * 4 + 0] = p1;
    vertices[vxlId * 4 + 1] = p2;
    vertices[vxlId * 4 + 2] = p3;
    vertices[vxlId * 4 + 3] = p4;
    
    pcl::Vertices v1, v2;
    uint32_t numPoints = vxlId * 4;
    v1.vertices = {numPoints, numPoints+1, numPoints+2};
    v2.vertices = {numPoints, numPoints+2, numPoints+3};
    polygons[vxlId * 2 + 0] = v1;
    polygons[vxlId * 2 + 1] = v2;
  }
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::distanceMapFromOccupancy(const Eigen::Vector3f& bbx_min, const Eigen::Vector3f& bbx_max, const uint16_t depth, const float max_distance, const bool inflate_obstacle_space)
{
  // Check that occupancy tree exists
  if (occupancyTree_.getTreeDepth() == 0)
  {
    std::cout << "[OccupancyMap::distanceMapFromOccupancy] occupancy tree is empty. Can't compute a distance map." << std::endl;
    return false;
  }
  
  // Delete distance map if it already exists
  if (distanceMap_)
  {
    distanceMap_->~DynamicEDT3D();
    delete distanceMap_;
    distanceMap_ = NULL;
  }
  
  // Check depth
  if (depth < 0 || depth > occupancyTree_.getTreeDepth())
  {
    std::cout << "[OccupancyMap::distanceMapFromOccupancy] depth must be greater than 0 and smaller than occupancy tree depth." << std::endl;
    return false;
  }
  
  distanceMapMaxDist_ = max_distance;
  depth_ = depth;
  dmVoxelSize_ = pow (2, occupancyTree_.getTreeDepth() - depth_);
  
  // Get bounding box size
  bbxMin_ = octomap::point3d (bbx_min[0], bbx_min[1], bbx_min[2]);
  bbxMax_ = octomap::point3d (bbx_max[0], bbx_max[1], bbx_max[2]);

  octomap::OcTreeKey bbxMinKey = occupancyTree_.coordToKey(bbxMin_, depth_);
  octomap::OcTreeKey bbxMaxKey = occupancyTree_.coordToKey(bbxMax_, depth_);
  octToDmOffset_ = bbxMinKey;  
  
  // Initialize an obstacle map
  int sizeX = (bbxMaxKey[0] / dmVoxelSize_) - (bbxMinKey[0] / dmVoxelSize_) + 1;
  int sizeY = (bbxMaxKey[1] / dmVoxelSize_) - (bbxMinKey[1] / dmVoxelSize_) + 1;
  int sizeZ = (bbxMaxKey[2] / dmVoxelSize_) - (bbxMinKey[2] / dmVoxelSize_) + 1;
  
  obstacleMap_ = new bool**[sizeX];
  # pragma omp parallel for    
  for (int x=0; x<sizeX; x++) {
    obstacleMap_[x] = new bool*[sizeY];
    for (int y=0; y<sizeY; y++) {
      obstacleMap_[x][y] = new bool[sizeZ];
      for(int z=0; z<sizeZ; z++)
        obstacleMap_[x][y][z] = false;
    }
  }

  // Fill occupancy data
  const std::vector<std::vector<int> > voxelNeighborhood = createVoxelNeighborhood6();
  # pragma omp parallel for
  for(int dx=0; dx<sizeX; dx++)
  {
    for(int dy=0; dy<sizeY; dy++)
    {
      for(int dz=0; dz<sizeZ; dz++)
      {
        octomap::OcTreeKey key = distanceMapKeyToOccupancyTreeKey(octomap::OcTreeKey(dx, dy, dz));
        octomap::OcTreeNode *node = occupancyTree_.search(key, depth);
        
        // If voxel is occupied or occluded add it to obstacle space
        if(!node || (node && occupancyTree_.isNodeOccupied(node)))
        {
          obstacleMap_[dx][dy][dz] = true;
        }
        
        // If voxel is free and one of it's neighbors is occluded - add it to obstacle space
        else if (inflate_obstacle_space)
        {
          for (size_t nbrId = 0; nbrId < voxelNeighborhood.size(); nbrId++)
          {
            int nbrDx = dx + voxelNeighborhood[nbrId][0];
            int nbrDy = dy + voxelNeighborhood[nbrId][1];
            int nbrDz = dz + voxelNeighborhood[nbrId][2];
            
            octomap::OcTreeKey nbr_oct_key = distanceMapKeyToOccupancyTreeKey(octomap::OcTreeKey(nbrDx, nbrDy, nbrDz));
            octomap::OcTreeNode *nbr_node = occupancyTree_.search(nbr_oct_key, depth_);
            
            if (!nbr_node)
            {
              obstacleMap_[dx][dy][dz] = true;
              continue;
            }
          }            
        }
      }
    }
  }
  
  // Construct distance map
  float cellMaxDistSquared = pow  ( std::ceil(max_distance / (occupancyTree_.getResolution() * static_cast<float>(dmVoxelSize_))), 2);
  distanceMap_ = new DynamicEDT3D (static_cast<int>(cellMaxDistSquared));
  distanceMap_->initializeMap (sizeX, sizeY, sizeZ, obstacleMap_);
  distanceMap_->update(true);
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::setBoundingPlanes (const std::vector<Eigen::Vector4f> &bounding_planes, const std::vector<float> &offsets)
{
  if (offsets.size() == 0)
  {
    boundingPlanes_ = bounding_planes;
  }
  else if (bounding_planes.size() != offsets.size())
  {
    std::cout << "[OccupancyMap::setBoundingPlanes] bounding plane vector and offset vector have different sizes." << std::endl;
    return false;
  }
  else
  {
    boundingPlanes_.resize(bounding_planes.size());
    
    for (size_t planeId = 0; planeId < offsets.size(); planeId++)
    {
      Eigen::Vector3f pTMP, nTMP;
      utl::planeCoefficientsToPointNormal(bounding_planes[planeId], pTMP, nTMP);
      pTMP += nTMP * offsets[planeId];
      utl::pointNormalToPlaneCoefficients(pTMP, nTMP, boundingPlanes_[planeId]);
    }
  }
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
float OccupancyMap::getPointBoundingPlaneMinSignedDistance(const Eigen::Vector3f& point) const
{
  // Check bounding planes first
  float planeDistance = std::numeric_limits<float>::max();
  
  for (size_t planeId = 0; planeId < boundingPlanes_.size(); planeId++)
    planeDistance = std::min(planeDistance, utl::pointToPlaneSignedDistance(point, boundingPlanes_[planeId]));
  
  return planeDistance;
}

////////////////////////////////////////////////////////////////////////////////
float OccupancyMap::getNearestOccludedOccupiedDistance(const Eigen::Vector3f& point) const
{
  octomap::OcTreeKey key_dm = getDistanceMapKey(point);
  float distance = distanceMap_->getDistance(key_dm[0], key_dm[1], key_dm[2]) * occupancyTree_.getResolution() * static_cast<float>(dmVoxelSize_);
  if (distance < 0)
    return distanceMapMaxDist_;
  
  return distance;
}

////////////////////////////////////////////////////////////////////////////////
float OccupancyMap::getNearestObstacleDistance(const Eigen::Vector3f& point) const
{
  // Check that distance map was initialized
  if (!distanceMap_)
  {
    std::cout << "[OccupancyMap::getNearestObstacleDistance] distance map was not initialized." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }

  // Calculate distance to the occluded/occupied cell
  float cellDistance = getNearestOccludedOccupiedDistance(point);
  
  // Calculate distance to the bounding planes
  float planeDistance = getPointBoundingPlaneMinSignedDistance(point);
  planeDistance = std::max(-planeDistance, 0.0f);
    
  // Otherwise return cell distance
  return std::max(cellDistance, planeDistance);
}

////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f OccupancyMap::getNearestObstacle(const Eigen::Vector3f& point) const
{
  // Check that distance map was initialized
  if (!distanceMap_)
  {
    std::cout << "[OccupancyMap::getNearestObstacle] distance map was not initialized." << std::endl;
    return Eigen::Vector3f::Ones() * std::numeric_limits<float>::quiet_NaN();
  }
  
  // Get the distance to the nearest bounding plane
  float planeDistance = 0;
  float nearestPlaneId = 0;
  for (size_t planeId = 0; planeId < boundingPlanes_.size(); planeId++)
  {
    float curDistance = utl::pointToPlaneSignedDistance(point, boundingPlanes_[planeId]);
    if (curDistance < planeDistance)
    {
      planeDistance = curDistance;
      nearestPlaneId = planeId;
    }
  }
  planeDistance = std::abs(planeDistance);
  
  // Get the distance to the nearest occluded/occupied cell
  octomap::OcTreeKey key_dm = getDistanceMapKey(point);
  float cellDistance = distanceMap_->getDistance(key_dm[0], key_dm[1], key_dm[2]) * occupancyTree_.getResolution() * static_cast<float>(dmVoxelSize_);

  // Get the closest point
  Eigen::Vector3f nearestPoint;
  if (cellDistance > planeDistance)
  {
    IntPoint3D closest_point_dm = distanceMap_->getClosestObstacle(key_dm[0], key_dm[1], key_dm[2]);
    octomap::OcTreeKey closest_key_dm(closest_point_dm.x, closest_point_dm.y, closest_point_dm.z);
    octomap::OcTreeKey closest_key_oct = distanceMapKeyToOccupancyTreeKey(closest_key_dm);
    octomap::point3d closestPoint_oct = occupancyTree_.keyToCoord(closest_key_oct, depth_);    
    nearestPoint = Eigen::Vector3f (closestPoint_oct.x(), closestPoint_oct.y(), closestPoint_oct.z());
  }
  else
  {
    nearestPoint = utl::projectPointToPlane<float>(point, boundingPlanes_[nearestPlaneId]);
  }
  
  return nearestPoint;
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::isPointOccluded(const Eigen::Vector3f& point) const
{
  // Check that distance map was initialized
  if (!distanceMap_)
  {
    std::cout << "[OccupancyMap::getPointDistance] distance map was not initialized." << std::endl;
    return std::numeric_limits<float>::quiet_NaN();
  }
  
  octomap::OcTreeKey key = getOccupanyTreeKey(point);
  octomap::OcTreeNode *node = occupancyTree_.search(key, depth_);
  return !node;
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::getOccludedSpaceBoundaryMesh (pcl::PointCloud<pcl::PointXYZ> &vertices, std::vector<pcl::Vertices> &polygons) const
{
  // Check that grid map was initialized
  if (!obstacleMap_ || occupancyTree_.getTreeDepth() == 0)
  {
    std::cout << "[OccupancyMap::getOccludedSpaceBoundaryMesh] either occupancy tree or distance map have not been initialized." << std::endl;
    return false;
  }
  
  // Clear output pointcloud
  vertices.resize(0);
  polygons.resize(0);
  
//   double start = pcl::getTime ();

  // Get a list of voxel neigbors on the boundary
  std::vector<std::pair<octomap::point3d, int> > voxelNeighborList;
  std::mutex voxelNeighborListMutex;
  const std::vector<std::vector<int> > voxelNeighborhood = createVoxelNeighborhood6();
  
  #pragma omp parallel for
  for(int dx=1; dx<distanceMap_->getSizeX()-1; dx++)
  {
    for(int dy=1; dy<distanceMap_->getSizeY()-1; dy++)
    {
      for(int dz=1; dz<distanceMap_->getSizeZ()-1; dz++)
      {
        // Check if this is an occluded voxel
        octomap::OcTreeKey oct_key = distanceMapKeyToOccupancyTreeKey(octomap::OcTreeKey(dx, dy, dz));
        octomap::OcTreeNode *node = occupancyTree_.search(oct_key, depth_);
        if (!node)
        {
          // Add polygons for each neighbor voxel that doesn't belong to the occluded space
          for (size_t nbrId = 0; nbrId < voxelNeighborhood.size(); nbrId++)
          {
            int nbrDx = dx + voxelNeighborhood[nbrId][0];
            int nbrDy = dy + voxelNeighborhood[nbrId][1];
            int nbrDz = dz + voxelNeighborhood[nbrId][2];
            
            octomap::OcTreeKey nbr_oct_key = distanceMapKeyToOccupancyTreeKey(octomap::OcTreeKey(nbrDx, nbrDy, nbrDz));
            octomap::OcTreeNode *nbr_node = occupancyTree_.search(nbr_oct_key, depth_);
            octomap::point3d pointOct = occupancyTree_.keyToCoord(oct_key, depth_);
            
            if (nbr_node)
            {
              voxelNeighborListMutex.lock();
              voxelNeighborList.push_back(std::make_pair(pointOct, nbrId));
              voxelNeighborListMutex.unlock();
            }
          }
        }
      }
    }
  }
    
  // Generate mesh
  generateVoxelBoundaryMesh(voxelNeighborList, vertices, polygons);
    
//  std::cout << "OCCLUDED SPACE TIME: " << (pcl::getTime() - start) << " seconds." << std::endl;
 
// An alternative approach is to loop through the leafs of the occupancy tree directly.
// This is difficult because not all of the leaves are at the max depth. It is very 
// dificult to find neighbors of a leaf at non-max depth, because it's own neighbors
// may be at a higher depth.  
//   for(auto leafIt = occupancyTree_.begin_leafs_bbx(bbxMin_, bbxMax_, depth_), leafEnd=occupancyTree_.end_leafs_bbx(); leafIt!= leafEnd; leafIt++)
//   {
//     // If it is not occupied - skip it
//     if (occupancyTree_.isNodeOccupied(*leafIt))
//     {
//       int leafDepth = leafIt.getDepth();
//       octomap::OcTreeKey keyOct = leafIt.getKey();
//       octomap::point3d pointOct = occupancyTree_.keyToCoord(keyOct, leafDepth);
//       for (size_t nbrId = 0; nbrId < voxelNeighborhood_.size(); nbrId++)
//       {
//         octomap::OcTreeKey keyNbrOct = keyOct;
//         keyNbrOct[0] += voxelNeighborhood_[nbrId][0] * pow (2, occupancyTree_.getTreeDepth() - leafDepth);
//         keyNbrOct[1] += voxelNeighborhood_[nbrId][1] * pow (2, occupancyTree_.getTreeDepth() - leafDepth);
//         keyNbrOct[2] += voxelNeighborhood_[nbrId][2] * pow (2, occupancyTree_.getTreeDepth() - leafDepth);
//         octomap::OcTreeNode *nbr_node = occupancyTree_.search(keyNbrOct, leafDepth);
//         if (!nbr_node || !occupancyTree_.isNodeOccupied(nbr_node))
//         {
//           voxelNeighborList.push_back(std::make_tuple(pointOct, leafDepth, nbrId));
//         }
//       }
//     }
//   }  
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool OccupancyMap::getObstacleSpaceBoundaryMesh (pcl::PointCloud<pcl::PointXYZ> &vertices, std::vector<pcl::Vertices> &polygons) const
{
  // Check that obstacle map was initialized
  if (!obstacleMap_ || occupancyTree_.getTreeDepth() == 0)
  {
    std::cout << "[OccupancyMap::getOccludedOccupiedSpaceCloud] either occupancy tree or distance map have not been initialized." << std::endl;
    return false;
  }
  
  // Clear output pointcloud
  vertices.resize(0);
  polygons.resize(0);
  
//   double start = pcl::getTime ();
  
  // Get a list of voxel neigbors on the boundary
  std::vector<std::pair<octomap::point3d, int> > voxelNeighborList;
  std::mutex voxelNeighborListMutex;
  const std::vector<std::vector<int> > voxelNeighborhood = createVoxelNeighborhood6();
  
  #pragma omp parallel for
  for(size_t dx=1; dx<distanceMap_->getSizeX()-1; dx++) {
    for(size_t dy=1; dy<distanceMap_->getSizeY()-1; dy++) {
      for(size_t dz=1; dz<distanceMap_->getSizeZ()-1; dz++) {
        if (obstacleMap_[dx][dy][dz])
        {
          octomap::OcTreeKey oct_key = distanceMapKeyToOccupancyTreeKey(octomap::OcTreeKey(dx, dy, dz));
          octomap::point3d pointOct = occupancyTree_.keyToCoord(oct_key, depth_);
          
          // Add polygons for each neighbor voxel that doesn't belong to the occluded space
          for (size_t nbrId = 0; nbrId < voxelNeighborhood.size(); nbrId++)
          {
            int nbrDx = dx + voxelNeighborhood[nbrId][0];
            int nbrDy = dy + voxelNeighborhood[nbrId][1];
            int nbrDz = dz + voxelNeighborhood[nbrId][2];
            
            if (!obstacleMap_[nbrDx][nbrDy][nbrDz])
            {
              voxelNeighborListMutex.lock();
              voxelNeighborList.push_back(std::make_pair(pointOct, nbrId));
              voxelNeighborListMutex.unlock();
            }
          }
        }
      }
    }
  }
  
  // Generate mesh
  generateVoxelBoundaryMesh(voxelNeighborList, vertices, polygons);
  
//   std::cout << "OBSTACLE SPACE TIME: " << (pcl::getTime() - start) << " seconds." << std::endl;
  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::showObstacleSpaceBoundaryMesh  ( pcl::visualization::PCLVisualizer &visualizer, const std::string &id) const
{
  pcl::PointCloud<pcl::PointXYZ> cloudMesh;
  std::vector<pcl::Vertices> polygonsMesh;
  getObstacleSpaceBoundaryMesh(cloudMesh, polygonsMesh);
  visualizer.addPolygonMesh<pcl::PointXYZ>(cloudMesh.makeShared(), polygonsMesh, id);
}

////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::showOccludedSpaceBoundaryMesh  ( pcl::visualization::PCLVisualizer &visualizer, const std::string &id) const
{
  pcl::PointCloud<pcl::PointXYZ> cloudMesh;
  std::vector<pcl::Vertices> polygonsMesh;
  getOccludedSpaceBoundaryMesh(cloudMesh, polygonsMesh);
  visualizer.addPolygonMesh<pcl::PointXYZ>(cloudMesh.makeShared(), polygonsMesh, id);
}

typedef boost::shared_ptr<OccupancyMap> OccupancyMapPtr;
typedef boost::shared_ptr<const OccupancyMap> OccupancyMapConstPtr;

#endif    // OCCUPANCY_MAP_HPP