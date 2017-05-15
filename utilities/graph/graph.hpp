// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_UTILITIES_HPP
#define GRAPH_UTILITIES_HPP

// Utilities includes
#include <graph/graph_base.hpp>
#include <graph/graph_primitives.hpp>

namespace utl
{
  /** \brief Data structure representing an undirected unwheighted graph.
   * Self loops are not allowed.
   */
  class Graph : public utl::GraphBase<Vertex, Edge>
  {
  public:
    
    /** \brief Empty constructor. */
    Graph ()  :
      utl::GraphBase< utl::Vertex, utl::Edge >()
    { }
    
    /** \brief Constructor that preallocates memory for vertices. */
    Graph (const int num_vertices)  :
      utl::GraphBase< utl::Vertex, utl::Edge >(num_vertices)
    { }

    /** \brief Destructor. */
    ~Graph () {};
  };
}

#endif  // GRAPH_UTILITIES_HPP