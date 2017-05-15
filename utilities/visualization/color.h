// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

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
