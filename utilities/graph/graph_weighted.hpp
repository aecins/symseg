// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_WEIGHTED_UTILITIES_HPP
#define GRAPH_WEIGHTED_UTILITIES_HPP

// Utilities includes
#include <graph/graph_base.hpp>
#include <graph/graph_primitives.hpp>

namespace utl
{
  /** \brief Data structure representing an undirected unwheighted graph.
   * Self loops are not allowed.
   */
  class GraphWeighted : public utl::GraphBase<Vertex, EdgeWeighted>
  {      
  public:
    
    using utl::GraphBase<Vertex, EdgeWeighted>::getEdge;
    using utl::GraphBase<Vertex, EdgeWeighted>::addEdge;
    
    /** \brief Empty constructor. */
    GraphWeighted ()  :
      utl::GraphBase< utl::Vertex, utl::EdgeWeighted >()
    { }
    
    /** \brief Constructor that preallocates memory for vertices. */
    GraphWeighted (const int num_vertices)  :
      utl::GraphBase< utl::Vertex, utl::EdgeWeighted >(num_vertices)
    { }

    /** \brief Destructor. */
    ~GraphWeighted () {};      
    
    /** \brief Add an edge to the graph if it doesn't exist already.
     *  \param[in] v1     index of first vertex
     *  \param[in] v2     index of second vertex
     *  \param[in] weight edge weight
     *  \return false if graph is not consistent
    */
    inline bool
    addEdge(const int vtx1_id, const int vtx2_id, const float weight)
    {
      return utl::GraphBase<Vertex, EdgeWeighted>::addEdge(EdgeWeighted (vtx1_id, vtx2_id, weight));
    }
    
    /** \brief Get edge at specified index.
     *  \param[in]  edge_id   edge index
     *  \param[out] vtx1_id   index of first edge
     *  \param[out] vtx2_id   index of first edge
     *  \param[out] weight    weight of the edge
     *  \return FALSE if requested edge id is greater than number of edges.
     */
    inline bool
    getEdge (const int edge_id, int &vtx1_id, int &vtx2_id, float &weight) const
    {
      if (edge_id >= getNumEdges())
      {
        std::cout << "[utl::Graph::getEdge] requested edge is out of bounds." << std::endl;
        return false;
      }
      
      EdgeWeighted edge;
      bool success = utl::GraphBase<Vertex, EdgeWeighted>::getEdge(edge_id, edge);
      
      if (success)
      {
        vtx1_id = edge.vtx1Id_;
        vtx2_id = edge.vtx2Id_;
        weight  = edge.weight_;
      }

      return success;
    }
    
    /** \brief Set the weight of an edge. If it doesn't exist - return false.
     *  \param[in] v1     index of first vertex
     *  \param[in] v2     index of second vertex
     *  \param[in] weight edge weight
     *  \return false if edge does not exist
     */
    inline bool
    setEdgeWeight(const int vtx1_id, const int vtx2_id, const float weight)
    {
      // NOTE: this should be replaced by the getEdge method
      // Check if v1 and v2 already exist
      int vtx1NbrIt, vtx2NbrIt;
      if (!getEdgeNeighborListPositions(vtx1_id, vtx2_id, vtx1NbrIt, vtx2NbrIt))
        return false;
              
      // If the edge doesn't exist - return false
      if (vtx1NbrIt == -1 && vtx2NbrIt == -1)
      {
        std::cout << "[utl::GraphWeighted::setEdgeWeight] edge does not exist in the graph (vtx1: " << vtx1_id << ", vtx2: " << vtx2_id << ")." << std::endl;
        return false;
      }
      
      // If edge it already exists - set it's weight
      int edgeId = vertex_list_[vtx1_id].neighbor_edges_[vtx2NbrIt];
      edge_list_[edgeId].weight_ = weight;
              
      return true;
    }
    
    /** \brief Set the weight of an edge. If it doesn't exist - return false.
     *  \param[in] edge_id  index of the edge
     *  \param[in] weight edge weight
     *  \return false if edge does not exist
     */
    inline bool
    setEdgeWeight(const int edge_id, const float weight)
    {
      if (edge_id > getNumEdges())
      {
        std::cout << "[utl::GraphWeighted::setEdgeWeight] edge does not exist in the graph (edge id: " << edge_id << ", num edges: " << getNumEdges() << ")." << std::endl;
        return false;
      }
        
      // If edge it already exists - set it's weight
      edge_list_[edge_id].weight_ = weight;
              
      return true;
    }
    
    /** \brief Get the weight of an edge. If it doesn't exist - return false.
     *  \param[in]  v1     index of first vertex
     *  \param[in]  v2     index of second vertex
     *  \param[out] weight edge weight
     *  \return false if edge does not exist
     */
    inline bool
    getEdgeWeight(const int vtx1_id, const int vtx2_id, float &weight)
    {
      // NOTE: this should be replaced by the getEdge method
      // Check if v1 and v2 already exist
      int vtx1NbrIt, vtx2NbrIt;
      if (!getEdgeNeighborListPositions(vtx1_id, vtx2_id, vtx1NbrIt, vtx2NbrIt))
        return false;
              
      // If the edge doesn't exist - return false
      if (vtx1NbrIt == -1 && vtx2NbrIt == -1)
      {
        std::cout << "[utl::GraphWeighted::getEdgeWeight] edge does not exist in the graph (vtx1: " << vtx1_id << ", vtx2: " << vtx2_id << ")." << std::endl;
        return false;
      }

      // If edge it already exists - get it's weight
      int edgeId = vertex_list_[vtx1_id].neighbor_edges_[vtx2NbrIt];
      weight = edge_list_[edgeId].weight_;

      return true;
    }
    
    /** \brief Get the weight of an edge. If it doesn't exist - return false.
     *  \param[in] edge_id  index of the edge
     *  \param[in] weight edge weight
     *  \return false if edge does not exist
     */
    inline bool
    getEdgeWeight(const int edge_id, float &weight)
    {
      if (edge_id > getNumEdges())
      {
        std::cout << "[utl::GraphWeighted::setEdgeWeight] edge does not exist in the graph (edge id: " << edge_id << ", num edges: " << getNumEdges() << ")." << std::endl;
        return false;
      }
        
      // If edge it already exists - set it's weight
      weight = edge_list_[edge_id].weight_;

      return true;
    }      
  };
}

#endif  // GRAPH_WEIGHTED_UTILITIES_HPP