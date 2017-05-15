// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_BASE_H
#define GRAPH_BASE_H

// STD includes
#include <iostream>
#include <vector>

namespace utl
{
  /** \brief Data structure representing a undirected acyclic graph. Self
   * loops are not allowed.
   * 
   * This class is templated on two parameters:
   * 1. <VertexT> is a struct used to store vertex infromation. It must have
   *    the following members:
   *      "neighbors_"      stores the indices of adjacent vertices
   *      "neighbor_edges"  stores the indices of edges corresponding to
   *                        adjacent vertices
   * 2. <EdgeT> is a struct storing the edge information. It must have members
   * "vtx1Id_" and "vtx2Id_" that hold the indices of the two vertices forming
   * an edge.
   * 
   * Internally graph information is stored in two vectors: a vector of 
   * <VertexT> and a vector of <EdgeT>. Vertex and edge indices correspond to
   * their positions in the vectors.
   */    
  template <typename VertexT, typename EdgeT>
  class GraphBase
  {
  public:
    
    /** \brief Empty constructor. */
    GraphBase ();
    
    /** \brief Constructor that preallocates memory for vertices. */
    GraphBase (const int num_vertices);

    /** \brief Destructor. */
    ~GraphBase ();
    
    /** \brief Add an edge to the graph if it doesn't exist already.
     *  \param[in]  edge  edge
     *  \return false if graph is not consistent
     */
    inline bool
    addEdge (const EdgeT &edge);
    
    /** \brief Add an edge to the graph if it doesn't exist already.
     *  \param[in] v1    index of first vertex
     *  \param[in] v2    index of second vertex
     *  \return false if graph is not consistent
     */
    inline bool
    addEdge (const int vtx1_id, const int vtx2_id);
    
    /** \brief Remove all vertices and edges from the graph. */
    inline void
    clear ();

    /** \brief Clear graph and preallocate memory for vertices in the
     * adjacency list. Internally this function first clears the graph and
     * then resizes the adjacency list to the required size.
     *  \param[in]  num_vertices  number of vertices in the graph
     *  \note Existing graph data will be erased.
     */
    inline void
    preallocateVertices (const int num_vertices);
    
    /** \brief Get number of vertices in the graph.  */
    inline int
    getNumVertices () const;

    /** \brief Get number of neighbors of a vertex. Return -1 if vertex with
     * requested id does not exist. */
    inline int
    getNumVertexNeighbors (const int vtx_id)  const;
    
    /** \brief Get number of edges in the graph.  */
    inline int
    getNumEdges ()  const;

    /** \brief Get vertex at specified index.
     *  \param[in]  vtx_id    vertex index
     *  \param[out] vertex    vertex
     *  \return FALSE if vertex with requested id doesn't exist in the graph.
     */
    inline bool
    getVertex (const int vtx_id, VertexT &vertex) const;

    /** \brief Get index of the neighbor vertex at a specified position in the
     * neighbor list.
     *  \param[in]  vtx_id      vertex index
     *  \param[in]  nbr_it  position of the neighbor in the neighbor list
     *  \param[out] nbr_id      neighbor vertex index
     *  \return FALSE if vertex with requested id doesn't exist in the graph
     *          or doesn't have a neighbor at specified position.
     */
    inline bool
    getVertexNeighbor  (const int vtx_id, const int nbr_it, int &nbr_id) const;
    
    /** \brief Get indices of the neighbors of a vertex at specified index.
     *  \param[in]  vtx_id    vertex index
     *  \param[out] neighbors neighbor indices
     *  \return FALSE if vertex with requested id doesn't exist in the graph.
     */
    inline bool
    getVertexNeighbors  (const int vtx_id, std::vector<int> &neighbors) const;
    
    /** \brief Get index of an edge between specified vertices.
     *  \param[in]  v1      index of first vertex
     *  \param[in]  v2      index of second vertex
     *  \param[out] edge_id edge index
     *  \return false if edge does not exist
     */
    inline bool
    getEdgeId (const int vtx1_id, const int vtx2_id, int edge_id) const;
    
    /** \brief Get edge at specified index.
     *  \param[in]  edge_id   edge index
     *  \param[out] edge      edge
     *  \return FALSE if edge with requested id doesn't exist in the graph.
     */
    inline bool
    getEdge (const int edge_id, EdgeT &edge) const;
          
    /** \brief Get edge between specified vertices.
     *  \param[in]  v1      index of first vertex
     *  \param[in]  v2      index of second vertex
     *  \param[out] edge    edge
     *  \return false if edge does not exist
     */
    inline bool
    getEdge(const int vtx1_id, const int vtx2_id, EdgeT &edge) const;
    
    /** \brief Get edge vertex indices.
     *  \param[in]  edge_id   edge index
     *  \param[out] vtx1_id   index of first edge
     *  \param[out] vtx2_id   index of first edge
     *  \return FALSE if edge with requested id doesn't exist in the graph.
     */
    inline bool
    getEdgeVertexIds (const int edge_id, int &vtx1_id, int &vtx2_id) const;      
    
    /** \brief Print the adjacency list of the graph. */
    inline void
    printAdjacencyList ()  const;

    /** \brief Print the edge list of the graph. */
    inline void
    printEdgeList ()  const;
          
//       /** \brief Check if the adjaceny list is consistent. Two properties are
//        * checked:
//        * 1. If an edge exists between two vertices, both vertices must have each
//        *    other's id in their respective neighbor lists.
//        * 2. There must be no self loops i.e. a vertex can not helf itself as a
//        *    neighbor.
//        *  \return TRUE if both of the above conditions are satisfied, FALSE otherwise.
//        */      
//       inline bool
//       checkConsistency  () const;      
    
  protected:
    
    /** \brief Vertex list. */
    std::vector<VertexT> vertex_list_;

    /** \brief Edge list. */
    std::vector<EdgeT> edge_list_;

    /** \brief Given a source vertex and a target vertex find the position of
     * the target vertex in the source vertex's neighbor list. If source
     * vertex does not exists or it's neighbor list does not contain the 
     * target vertex - return -1.
     *  \param[in]  src      index of the first vertex
     *  \param[in]  v2      index of the second vertex
     *  \param[in]  v1_pos  position of the first vertex in the second's vertex neigbhor list
     *  \param[in]  v2_pos  position of the second vertex in the first's vertex neigbhor list
     *  \return position of target vertex in source vertex's neighbor list
     */      
    inline int
    getVertexNeighborListPosition  (const int src_vtx_id, const int tgt_vtx_id)  const;
    
    /** \brief Given two vertices get their positions in each other's neighbor
     * lists. If the vertex isn't present in the other vertex's neighbor list
     * it's position is returned as -1.
     * There are three cases:
     * 1. Both positions are non-negative
     *    This means that there is an edge between the vertices.
     * 2. Both positions are -1.
     *    This means that there is no edge between the vertices.
     * 3. Once position is -1 and other one is non-negative
     *    This indicates that the adjacency list storing the grapg is not 
     *    consistent.
     *  \param[in]  v1      index of the first vertex
     *  \param[in]  v2      index of the second vertex
     *  \param[in]  v1_pos  position of the first vertex in the second's vertex neigbhor list
     *  \param[in]  v2_pos  position of the second vertex in the first's vertex neigbhor list
     *  \return FALSE if adjacency list is not consistent.
     */
    inline bool
    getEdgeNeighborListPositions  ( const int vtx1_id,
                                    const int vtx2_id,
                                    int &vtx1_nbr_it,
                                    int &vtx2_nbr_it
                                  )  const;
  };
}

#endif  // GRAPH_BASE_H