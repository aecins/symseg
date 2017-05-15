// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef MIN_CUT_HPP
#define MIN_CUT_HPP

// Boost includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

// Utilities includes
#include <graph/graph_weighted.hpp>

// Typedefs
// NOTE! This should be moved inside the functions, so that they don't spread these typedefs outside this function.
// The likely solution is to put the whole mincut algorithm into it's own class
typedef boost::adjacency_list_traits  < boost::vecS, boost::vecS, boost::directedS > Traits;
typedef boost::adjacency_list         < boost::vecS,                                                                            // Container used for vertices
                                        boost::vecS,                                                                            // Container used for edges
                                        boost::directedS,                                                                       // Directional graph
                                        boost::property < boost::vertex_name_t, std::string,                                    // Vertex properties
                                        boost::property < boost::vertex_index_t, long,
                                        boost::property < boost::vertex_color_t, boost::default_color_type,
                                        boost::property < boost::vertex_distance_t, long,
                                        boost::property < boost::vertex_predecessor_t, Traits::edge_descriptor > > > > >,

                                        boost::property < boost::edge_capacity_t, float,                                       // Edge properties
                                        boost::property < boost::edge_residual_capacity_t, float,
                                        boost::property < boost::edge_reverse_t, Traits::edge_descriptor > > > > GraphBoost;

typedef boost::property_map< GraphBoost, boost::edge_capacity_t >::type CapacityMap;
typedef boost::property_map< GraphBoost, boost::edge_reverse_t>::type ReverseEdgeMap;
typedef boost::property_map< GraphBoost, boost::vertex_color_t, boost::default_color_type>::type VertexColorMap;

////////////////////////////////////////////////////////////////////////////////
bool addEdge (Traits::vertex_descriptor &v1, Traits::vertex_descriptor &v2, GraphBoost &graph, const float weight, CapacityMap &capacity_map, ReverseEdgeMap &reverse_edge_map)
{
  Traits::edge_descriptor edge, reverse_edge;
  bool edge_was_added, reverse_edge_was_added;

  boost::tie (edge, edge_was_added) = boost::add_edge ( v1, v2, graph );
  boost::tie (reverse_edge, reverse_edge_was_added) = boost::add_edge ( v2, v1, graph );
  if ( !edge_was_added || !reverse_edge_was_added )
    return (false);
  
  capacity_map[edge] = weight;
  capacity_map[reverse_edge] = weight;
  reverse_edge_map[edge] = reverse_edge;
  reverse_edge_map[reverse_edge] = edge;
  
  return true;
}
                                    
namespace utl
{
  /** \brief Perform a min cut on a graph
   *  \param[in]  source_potentials   weights between nodes and source node
   *  \param[in]  sink_potentials     weights between nodes and sink node
   *  \param[in]  binary_potentials   binary potential structure and weights
   *  \param[out] source_points       points belonging to source
   *  \param[out] sink_points         points belonging to sink
   */
  double mincut ( const std::vector<float> &source_potentials, 
                  const std::vector<float> &sink_potentials,
                  const utl::GraphWeighted &binary_potentials,
                  std::vector<int> &source_points,
                  std::vector<int> &sink_points
                )
  {
    ////////////////////////////////////////////////////////////////////////////
    // Check input
    if (! (   (source_potentials.size() == sink_potentials.size()) && 
              (source_potentials.size() == binary_potentials.getNumVertices())))
    {
      std::cout << "[utl::minCut] number of vertices in source potentials, sink potentials and binary potentials are not equal." << std::endl;
      return -1.0;
    }
    
    ////////////////////////////////////////////////////////////////////////////
    // Build graph
    
    int numVertices = source_potentials.size();
    GraphBoost graph;
    std::vector< Traits::vertex_descriptor > vertices;
    Traits::vertex_descriptor source;
    Traits::vertex_descriptor sink;
    CapacityMap capacity          = boost::get (boost::edge_capacity, graph);
    ReverseEdgeMap reverseEdgeMap = boost::get (boost::edge_reverse, graph);
    VertexColorMap vertexColorMap = boost::get (boost::vertex_color, graph);

      // Add vertices
    vertices.resize(numVertices + 2);
    for (size_t i = 0; i < static_cast<size_t>(numVertices + 2); i++)
      vertices[i] = boost::add_vertex(graph);
    
    source  = vertices[source_potentials.size()];
    sink    = vertices[source_potentials.size()+1];
    
    // Add source and sink edges
    for (size_t i = 0; i < static_cast<size_t>(numVertices); i++)
    {
      addEdge(vertices[i], source, graph, source_potentials[i], capacity, reverseEdgeMap);
      addEdge(vertices[i], sink,   graph, sink_potentials[i], capacity, reverseEdgeMap);
    }
    
    // Add binary edges
    for (size_t edgeId = 0; edgeId < binary_potentials.getNumEdges(); edgeId++)
    {
      // Get edge information
      int vtx1Id, vtx2Id;
      float weight;
      
      if (!binary_potentials.getEdge(edgeId, vtx1Id, vtx2Id, weight))
      {
        std::cout << "[utl::minCut] could not add binary edges to Boost graph." << std::endl;
        abort();        
      }
      
      // Add it to Boost graph
      Traits::vertex_descriptor v1 = vertices[vtx1Id];
      Traits::vertex_descriptor v2 = vertices[vtx2Id];
      addEdge(v1, v2, graph, weight, capacity, reverseEdgeMap);      
    }
        
    ////////////////////////////////////////////////////////////////////////////
    // Compute maximim flow
    
    double flow = boost::boykov_kolmogorov_max_flow(graph, source, sink);
    
    ////////////////////////////////////////////////////////////////////////////
    // Find foreground and background points
    
    source_points.clear();
    sink_points.clear();
    
    for (size_t i = 0; i < static_cast<size_t>(numVertices); i++)
    {    
      if (vertexColorMap(vertices[i]) == 0)
        source_points.push_back(i);
      else
        sink_points.push_back(i);
    }
    
    return flow;
  }
}

# endif // MIN_CUT_HPP