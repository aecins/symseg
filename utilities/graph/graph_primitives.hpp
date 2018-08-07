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
