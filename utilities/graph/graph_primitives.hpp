// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef GRAPH_PRIMITIVES_HPP
#define GRAPH_PRIMITIVES_HPP

namespace utl
{
  /** \brief Vertex struct. Holds the indices of all of it's neighboring 
   * vertices.
   */
  struct Vertex
  {
    /** \brief Empty constructor. */
    Vertex () :
      neighbors_ (0),
      neighbor_edges_ (0)
    { };
    
    /** \brief Vertex neighbor indices. */
    std::vector<int> neighbors_;
    
    /** \brief Indices of the corresponding edges. */
    std::vector<int> neighbor_edges_;
    
    /** \brief Print neighbor information. */
    void print () const
    {
      for (size_t nbrId = 0; nbrId < neighbors_.size(); nbrId++)
        std::cout << neighbors_[nbrId] << ", ";
      std::cout << std::endl;
    }
  };    
  
  /** \brief Edge struct. Holds indices of the vertices forming the edge. */
  struct Edge
  {
    /** \brief Empty constructor. */
    Edge () :
      vtx1Id_ (0),
      vtx2Id_ (0)
    { };
    
    /** \brief Constructor with vertex initialized. */
    Edge (const int vtx1_id, const int vtx2_id) :
      vtx1Id_(vtx1_id),
      vtx2Id_(vtx2_id)
    { };
    
    /** \brief Vertex indices. */
    int vtx1Id_;
    int vtx2Id_;
          
    /** \brief Print edge information. */
    void print () const
    {
      std::cout << vtx1Id_ << " <-> " << vtx2Id_ << std::endl;
    }
  };

  /** \brief Weighted edge struct. Holds indices of the vertices forming the
   * edge and the weight of the edge.
   */
  struct EdgeWeighted : public Edge
  {
    /** \brief Empty constructor. */
    EdgeWeighted () :
      Edge (),
      weight_ (0)
    { };
    
    /** \brief Constructor with vertex initialized. */
    EdgeWeighted (const int vtx1_id, const int vtx2_id, const float weight) :
      Edge (vtx1_id, vtx2_id),
      weight_(weight)
    { };
          
    /** \brief Edge weight. */
    float weight_;
          
    /** \brief Print edge information. */
    void print () const
    {
      std::cout << vtx1Id_ << " <-> " << vtx2Id_ << " (" << weight_ << ")" << std::endl;
    }      
  };    
}

#endif    //  GRAPH_PRIMITIVES_HPP
