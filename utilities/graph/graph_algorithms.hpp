/*****************************************************************************/
/*  Copyright (c) 2017, Aleksandrs Ecins                                     */
/*  All rights reserved.                                                     */
/*                                                                           */
/*  Redistribution and use in source and binary forms, with or without       */
/*  modification, are permitted provided that the following conditions       */
/*  are met:                                                                 */
/*                                                                           */
/*  1. Redistributions of source code must retain the above copyright        */
/*  notice, this list of conditions and the following disclaimer.            */
/*                                                                           */
/*  2. Redistributions in binary form must reproduce the above copyright     */
/*  notice, this list of conditions and the following disclaimer in the      */
/*  documentation and/or other materials provided with the distribution.     */
/*                                                                           */
/*  3. Neither the name of the copyright holder nor the names of its         */
/*  contributors may be used to endorse or promote products derived from     */
/*  this software without specific prior written permission.                 */
/*                                                                           */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS      */
/*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT        */
/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
/*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     */
/*  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
/*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         */
/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
/*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
/*****************************************************************************/

#ifndef GRAPH_ALGORITHMS_HPP
#define GRAPH_ALGORITHMS_HPP

// STD includes
#include <queue>

// Utilities includes
#include <graph/graph_base.hpp>
#include <std_vector.hpp>
#include <map.hpp>

namespace utl
{

  /** \brief Given a set of vertices in a graph return all edges between
   * the input set of vertices and the rest of the vertices in the graph.
   *  \param[in]  graph           graph
   *  \param[in]  cut_vertices    indices of vertices that were cut
   *  \param[in]  cut_edge_graph  a graph only containing the edges belonging to the cut
   */
  template <typename VertexT, typename EdgeT>
  inline void
  getCutEdges (const utl::GraphBase<VertexT, EdgeT> &graph, const std::vector<int> &cut_vertices, utl::GraphBase<VertexT, EdgeT> &cut_edge_graph)
  {
    cut_edge_graph.clear();
    
    // Loop over edges
    for (size_t edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
    {
      // Get the verices of the current edge
      EdgeT edge;
      graph.getEdge(edgeId, edge);
      int vtx1Id = edge.vtx1Id_;
      int vtx2Id = edge.vtx2Id_;
      
      // Find edge vertices are in the cut vertex set
      std::vector<int> vtx1Loc, vtx2Loc;
      int vtx1Count = utl::vectorFind(cut_vertices, vtx1Id, vtx1Loc);
      int vtx2Count = utl::vectorFind(cut_vertices, vtx2Id, vtx2Loc);
      
      // Add edge if one vertex is in cut vertices but the other is not
      if ((vtx1Count == 0 && vtx2Count > 0) || (vtx1Count > 0 && vtx2Count == 0))
        cut_edge_graph.addEdge(edge);
    }
  }
  
  /** \brief Find connected components in the graph.
   *  \param[in]  graph         graph object
   *  \param[in]  min_cc_size   minimum size of a valid connected component (default 0)
   *  \return    a vector of vectors where each inner vector corresponds to a 
   *             connected component and stores the indices of vertices belonging
   *             to it.
   */
  template <typename NeighborT, typename EdgeT>
  inline utl::Map
  getConnectedComponents  ( const utl::GraphBase<NeighborT, EdgeT> &graph, const int min_cc_size = 0)
  {
    std::vector<bool> visited (graph.getNumVertices(), false);
    utl::Map CCs;
    
    for (size_t vtxId = 0; vtxId < graph.getNumVertices(); vtxId++)
    {
      // If node has already been visited - skip
      if (visited[vtxId])
        continue;
      
      // Run breadth-first search from current vertex
      std::queue<int> vertexQueue;
      std::vector<int> CC;
      vertexQueue.push(vtxId);
      visited[vtxId] = true;
      
      while (!vertexQueue.empty())
      {
        // Get first vertex from the queue
        int curVtxId = vertexQueue.front();
        vertexQueue.pop();
        CC.push_back(curVtxId);
        
        // Loop over it's neighbors
        std::vector<int> neighbors;
        graph.getVertexNeighbors(curVtxId, neighbors);
        
        for (size_t nbrIt = 0; nbrIt < neighbors.size(); nbrIt++)
        {
          int nbrId = neighbors[nbrIt];
          
          if (!visited[nbrId])
          {
            vertexQueue.push(nbrId);
            visited[nbrId] = true;
          }
        }
      }
      if (static_cast<int>(CC.size()) > min_cc_size)
        CCs.push_back(CC);
    }
    
    return CCs;
  }
}
    
#endif    // GRAPH_ALGORITHMS_HPP