// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_BASE_HPP
#define GRAPH_BASE_HPP

// STD includes
#include <algorithm>

// Utilities includes
#include <graph/graph_base.h>

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
utl::GraphBase<VertexT, EdgeT>::GraphBase () :
  vertex_list_ (0),
  edge_list_(0)
{ }

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
utl::GraphBase<VertexT, EdgeT>::GraphBase (const int num_vertices) :
  vertex_list_ (num_vertices),
  edge_list_(0)
{ }

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
utl::GraphBase<VertexT, EdgeT>::~GraphBase ()
{ }

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::addEdge(const EdgeT &edge)
{
  int vtx1Id = edge.vtx1Id_;
  int vtx2Id = edge.vtx2Id_;
  
  // Check if v1 and v2 already exist
  int vtx1NBrIt, vtx2NbrIt;
  if (!getEdgeNeighborListPositions(vtx1Id, vtx2Id, vtx1NBrIt, vtx2NbrIt))
    return false;
  
  // Check that it's not a self loop
  if (vtx1Id == vtx2Id)
  {
    std::cout << "[utl::GraphBase::addEdge] Edge must be between two different vertices (" << vtx1Id << ", " << vtx2Id << " )." << std::endl;
    return false;
  }
  
  // Add edge if it doesn't exist yet
  if (vtx1NBrIt == -1 && vtx2NbrIt == -1)
  {
    // Increase graph size if needed
    int maxVId = std::max(vtx1Id, vtx2Id);
    if (getNumVertices() <= maxVId)
      vertex_list_.resize(maxVId+1);
    
    // Add edge to the adjacency list
    vertex_list_[vtx1Id].neighbors_.push_back(vtx2Id);
    vertex_list_[vtx1Id].neighbor_edges_.push_back(edge_list_.size());
    vertex_list_[vtx2Id].neighbors_.push_back(vtx1Id);
    vertex_list_[vtx2Id].neighbor_edges_.push_back(edge_list_.size());
    
    // Add edge to the edge list
    edge_list_.push_back(edge);
  }
//   else
//   {
//     std::cout << "[utl::GraphBase::addEdge] edge already exists (vtx1: " << vtx1Id << ", vtx2: " << vtx2Id << ")." << std::endl;
//   }
          
  return true;  
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::addEdge (const int vtx1_id, const int vtx2_id)
{
  EdgeT edge;
  edge.vtx1Id_ = vtx1_id;
  edge.vtx2Id_ = vtx2_id;
  return utl::GraphBase<VertexT, EdgeT>::addEdge(edge);
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline void utl::GraphBase<VertexT, EdgeT>::clear ()
{
  vertex_list_.resize(0);
  edge_list_.resize(0);
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline void utl::GraphBase<VertexT, EdgeT>::preallocateVertices (const int num_vertices)
{
  clear();
  vertex_list_.resize(num_vertices);
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline int utl::GraphBase<VertexT, EdgeT>::getNumVertices () const
{
  return vertex_list_.size();
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline int utl::GraphBase<VertexT, EdgeT>::getNumVertexNeighbors (const int vtx_id) const
{
  if (vtx_id < 0 || vtx_id >= getNumVertices())
    return -1;
  
  return vertex_list_[vtx_id].neighbors_.size();
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline int utl::GraphBase<VertexT, EdgeT>::getNumEdges () const
{
  return edge_list_.size();
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getVertex (const int vtx_id, VertexT &vertex) const
{
  if (vtx_id < 0 || vtx_id >= getNumVertices())
  {
    std::cout << "[utl::GraphBase::getVertex] requested vertex id is out of bounds ( vtx id: " << vtx_id << ")," << std::endl;
    return false;
  }
  
  vertex = vertex_list_[vtx_id];
  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getVertexNeighbor (const int vtx_id, const int nbr_it, int &nbr_id) const
{
  VertexT vertex;
  bool success = getVertex(vtx_id, vertex);
  
  if (success)
  {
    if (nbr_it < 0 || nbr_it >= vertex.neighbors_.size())
    {
      std::cout << "[utl::GraphBase::getVertexNeighbor] requested vertex neighbor position is out of bounds." << std::endl;
      std::cout << "[utl::GraphBase::getVertexNeighbor] ( vtx id: " << vtx_id << ", requested neigbor: " << nbr_it
                << ", num neighbors: " << vertex.neighbors_.size() << ")," << std::endl;
      return false;
    }
    
    nbr_id = vertex.neighbors_[nbr_it];
  }
  
  return success;
}


////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getVertexNeighbors (const int vtx_id, std::vector<int> &neighbors) const
{
  VertexT vertex;
  bool success = getVertex(vtx_id, vertex);
  
  if (success)
    neighbors = vertex.neighbors_;
  
  return success;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getEdgeId (const int vtx1_id, const int vtx2_id, int edge_id) const
{
  edge_id = -1;
  
  // Check if v1 and v2 exist
  int vtx1NbrIt, vtx2NbrIt;
  if (!getEdgeNeighborListPositions(vtx1_id, vtx2_id, vtx1NbrIt, vtx2NbrIt))
    return false;
          
  // If the edge doesn't exist - return false
  if (vtx1NbrIt == -1 && vtx2NbrIt == -1)
    return false;

  // If edge it already exists - get it's weight
  edge_id = vertex_list_[vtx1_id].neighbor_edges_[vtx2NbrIt];  
  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getEdge (const int edge_id, EdgeT &edge) const
{
  if (edge_id < 0 || edge_id >= getNumEdges())
  {
    std::cout << "[utl::GraphBase::getEdge] requested edge is out of bounds ( edge id: " << edge_id << ")," << std::endl;
    return false;
  }
  
  edge = edge_list_[edge_id];
  return true;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getEdge (const int vtx1_id, const int vtx2_id, EdgeT &edge) const
{
  int edgeId;
  bool success = utl::GraphBase<VertexT, EdgeT>::getEdgeId(vtx1_id, vtx2_id, edgeId);
  
  if (success)
    edge = edge_list_[edgeId];
  
  return success;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getEdgeVertexIds (const int edge_id, int &vtx1_id, int &vtx2_id) const
{
  EdgeT edge;
  bool success = utl::GraphBase<VertexT, EdgeT>::getEdge(edge_id, edge);
  
  if (success)
  {
    vtx1_id = edge.vtx1Id_;
    vtx2_id = edge.vtx2Id_;
  }

  return success;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline void utl::GraphBase<VertexT, EdgeT>::printAdjacencyList () const
{
  for (size_t vtx1Id = 0; vtx1Id < vertex_list_.size(); vtx1Id++)
  {
    std::cout << vtx1Id << " -> ";
    vertex_list_[vtx1Id].print();
  }
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline void utl::GraphBase<VertexT, EdgeT>::printEdgeList () const
{
  for (size_t edgeId = 0; edgeId < edge_list_.size(); edgeId++)
    edge_list_[edgeId].print();
}

// ////////////////////////////////////////////////////////////////////////////////
// template <typename VertexT, typename EdgeT>
// inline bool utl::GraphBase<VertexT, EdgeT>::checkConsistency () const
// {
//   // Create a boolean for every entry in the adjacency list
//   std::vector<std::vector<bool> > neighborsChecked (adjacency_list_.size());
//   for (size_t vtxId = 0; vtxId < adjacency_list_.size(); vtxId++)
//     neighborsChecked[vtxId].resize(adjacency_list_[vtxId].size(), false);
//   
//   // Loop over all edges and check that they are consistent with neighbors in the adjacency list
//   for (size_t edgeId = 0; edgeId < getNumEdges(); edgeId++)
//   {
//     int vtx1Id = edge_list_[edgeId].vtx1Id_;
//     int vtx2Id = edge_list_[edgeId].vtx2Id_;
//     
//     // Check self loop
//     if (vtx1Id == vtx2Id)
//       return false;
//     
//     // Check adjacency list consistency
//     int vtx1NbrIt, vtx2NbrIt;
//     if (!getEdgeNeighborListPositions(vtx1Id, vtx2Id, vtx1NbrIt, vtx2NbrIt))
//       return false;
//     
//     // Check that both neighbors corresponding to the edge exist
//     if (vtx1NbrIt == -1 || vtx2NbrIt == -1)
//     {
//       std::cout << "[utl::GraphBase::checkConsistency] edge with id " << edgeId << " doesn't have corresponding neighbors in the adjacency list." << std::endl;
//       return false;
//     }
//     
//     // Check that both neighbors have the correct edge id
//     if (adjacency_list_[vtx1Id][vtx2NbrIt].edgeId_ != edgeId || adjacency_list_[vtx2Id][vtx1NbrIt].edgeId_ != edgeId)
//     {
//       std::cout << "[utl::GraphBase::checkConsistency] neighbors found for edge with id " << edgeId << " have a different edge id on record ("
//                 << adjacency_list_[vtx1Id][vtx2NbrIt].edgeId_ << " and " << adjacency_list_[vtx2Id][vtx1NbrIt].edgeId_ <<  ")." << std::endl;
//       return false;
//     }
//     
//     // Update the checked adjacency list entry table
//     neighborsChecked[vtx1Id][vtx2NbrIt] = true;
//     neighborsChecked[vtx2Id][vtx1NbrIt] = true;
//   }
//   
//   // Check if there are any entries in the adjacency list that are not present in the edge list
//   for (size_t vtxId = 0; vtxId < neighborsChecked.size(); vtxId++)
//     for (size_t vtxNbrIt = 0; vtxNbrIt < neighborsChecked[vtxId].size(); vtxNbrIt++)
//       if (!neighborsChecked[vtxId][vtxNbrIt])
//       {
//         std::cout << "[utl::GraphBase::checkConsistency] there are neighbors in the adjacency list that do not correspond to any edge in the edge list." << std::endl;
//         std::cout << "[utl::GraphBase::checkConsistency] vtx: ";
//         adjacency_list_[vtxId][vtxNbrIt].print();
//         std::cout << std::endl;
//         return false;              
//       }
// }

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline int utl::GraphBase<VertexT, EdgeT>::getVertexNeighborListPosition  (const int src_vtx_id, const int tgt_vtx_id)  const
{
  // Get number of neighbors for input vertex
  int numNeighbors = getNumVertexNeighbors(src_vtx_id);
  
  // If vertex does not exist - return -1
  if (numNeighbors == -1)
    return -1;
  
  // Loop over source vertex neighbors and check if any of them is target vertex
  for (size_t nbrIt = 0; nbrIt < getNumVertexNeighbors(src_vtx_id); nbrIt++)
  {
    if (vertex_list_[src_vtx_id].neighbors_[nbrIt] == tgt_vtx_id)
      return nbrIt;
  }
  
  // If target vertex was not found - return -1
  return -1;
}

////////////////////////////////////////////////////////////////////////////////
template <typename VertexT, typename EdgeT>
inline bool utl::GraphBase<VertexT, EdgeT>::getEdgeNeighborListPositions  (const int vtx1_id, const int vtx2_id, int &vtx1_nbr_it, int &vtx2_nbr_it)  const
{
  // Get vertex neighbor list positions
  vtx1_nbr_it = getVertexNeighborListPosition(vtx2_id, vtx1_id);
  vtx2_nbr_it = getVertexNeighborListPosition(vtx1_id, vtx2_id);
  
  // Check adjacency list for consistency (DEBUG)
  if ((vtx1_nbr_it == -1 && vtx2_nbr_it != -1) || (vtx1_nbr_it != -1 && vtx2_nbr_it == -1))
  {
    std::cout << "[utl::GraphBase::getEdgeNeighborListPositions] one vertex has another as it's member, but not the other way around. Adjacency list is not consistent!" << std::endl;
    std::cout << "[utl::GraphBase::getEdgeNeighborListPositions] (vtx 1: " << vtx1_id << ", vtx 2: " << vtx2_id << ")" << std::endl;
    return false;
  }
  
  // Check that both neighbors have the same edge id (DEBUG)
  if (( vtx1_nbr_it != -1 && vtx2_nbr_it != -1) &&
      ( vertex_list_[vtx1_id].neighbor_edges_[vtx2_nbr_it] != vertex_list_[vtx2_id].neighbor_edges_[vtx1_nbr_it]))
  {
    std::cout << "[utl::GraphBase::getEdgeNeighborListPositions] neighbors belonging to the same edge have different edge ids!" << std::endl;
    std::cout << "[utl::GraphBase::getEdgeNeighborListPositions] ( vtx 1: " << vtx1_id << ", edge id: " << vertex_list_[vtx1_id].neighbor_edges_[vtx2_nbr_it] << ")" << std::endl;
    std::cout << "[utl::GraphBase::getEdgeNeighborListPositions] ( vtx 2: " << vtx2_id << ", edge id: " << vertex_list_[vtx2_id].neighbor_edges_[vtx1_nbr_it] << ")" << std::endl;
    return false;
  }        
  return true;
}

#endif  // GRAPH_BASE_HPP