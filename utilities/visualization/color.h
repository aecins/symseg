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

#ifndef PCL_VISUALIZATION_COLOR_H
#define PCL_VISUALIZATION_COLOR_H

#include <vector>

namespace utl
{
  struct Color
  {
    double r;
    double g;
    double b;
    
    Color ()
      : r (-1)
      , g (-1)
      , b (-1)
    {};
    
    Color (const double r, const double g, const double b)
      : r (r)
      , g (g)
      , b (b)
    {};
    
    std::vector<double> toStdVec ()
    {
      std::vector<double> c (3);
      c[0] = r; c[1] = g; c[2] = b;
      return c;
    }
    
    bool operator==(const Color& rhs) const
    {
      return (r == rhs.r && b == rhs.b && g == rhs.b);
    }
    
    bool operator!=(const Color& rhs) const
    {
      return (r != rhs.r && b != rhs.b && g != rhs.b);
    }
    
    template <typename Scalar>
    Color operator* (const Scalar x)
    {
      Color result;
        
      result.r = r * static_cast<double>(x);
      result.g = g * static_cast<double>(x);
      result.b = b * static_cast<double>(x);
      result.r = std::max(0.0, std::min(1.0, r));
      result.g = std::max(0.0, std::min(1.0, g));
      result.b = std::max(0.0, std::min(1.0, b));
      
      return result;
    }
  };
      
  typedef std::vector<Color> Colors;
}

#endif  // PCL_VISUALIZATION_COLOR_H
