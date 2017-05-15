// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef BRON_KERBOSCH_HPP
#define BRON_KERBOSCH_HPP

// Boost includes
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

// Utilities includes
#include <graph/graph.hpp>

struct CliqueVisitor
{
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> GraphBoost;
  typedef boost::property_map<GraphBoost, boost::vertex_index_t>::type IndexMap;
  
  // Constructor
  CliqueVisitor (IndexMap& index, std::list<std::list<int> >& cliques)
    : index_(index)
    , cliques_(cliques)
  { }

  // This function is called for every clique
  template <typename Clique, typename Graph>
  void clique(const Clique& c, const Graph& g)
  {   
    std::list<int> curClique;
    
    // Iterate over the clique
    for(typename Clique::const_iterator i = c.begin(); i != c.end(); ++i)
      curClique.push_back(index_[*i]);

    cliques_.push_back(curClique);
  }
  
  IndexMap& index_;
  std::list<std::list<int> >& cliques_;
};

namespace utl
{
  /** \brief Find all maximal cliques in a graph using the Bron-Kerbosch
   * algorithm.
   *  \param[in]  graph   input graph
   *  \param[out] cliques output cliques
   *  \param[in]  min_clique_size minimum size of a valid clique
   *  \note this is a wrapper around Bron-Kerbosch implementation in Boost library
   */
  int bronKerbosch  ( const utl::Graph &graph,
                      std::list<std::list<int> > &cliques,
                      const int min_clique_size = 2
                    )
  {
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> GraphBoost;
    typedef boost::property_map<GraphBoost, boost::vertex_index_t>::type IndexMap;
    
    // Convert to Boost graph
    GraphBoost graphBoost (graph.getNumVertices());
    IndexMap index = get(boost::vertex_index, graphBoost);

    for (int edgeId = 0; edgeId < graph.getNumEdges(); edgeId++)
    {
      utl::Edge edge;
      graph.getEdge(edgeId, edge);
      boost::add_edge(edge.vtx1Id_, edge.vtx2Id_, graphBoost);
    }
    
    // Find cliques
    CliqueVisitor visitor(index, cliques);
    boost::bron_kerbosch_all_cliques(graphBoost, visitor, min_clique_size); 
        
    return cliques.size();
  }
}

# endif // BRON_KERBOSCH_HPP