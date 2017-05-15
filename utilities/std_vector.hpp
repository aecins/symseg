// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef STD_VECTOR_UTILITIES_HPP
#define STD_VECTOR_UTILITIES_HPP

// STD includes
#include <iostream>
#include <fstream>
#include <assert.h>

namespace utl
{
  //--------------------------------------------------------------------------
  // Forward declarations
  //--------------------------------------------------------------------------

  /** \brief Get the maximum value of a vector of scalars
    *  \param[in] v  vector
    *  \return maximum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMax (const std::vector<Scalar> &v);
  
  /** \brief Get the minimum value of a vector of scalars
    *  \param[in] v  vector
    *  \return minimum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMin (const std::vector<Scalar> &v);
  
  //--------------------------------------------------------------------------
  // Vector sorting
  //--------------------------------------------------------------------------
  // NOTE: adapted from here: http://www.alecjacobson.com/weblog/?p=1527
  
  /** \brief Sorting modes */
  enum SortMode
  { 
    UNSORTED,     /**< unsorted */
    ASCENDING,    /**< ascending order */
    DESCENDING    /**< descending order */
  };

  
  /** \brief  Comparison struct used by sort for ascending order
    * \note: http://bytes.com/topic/c/answers/132045-sort-get-index
    */
  template<class T> struct index_cmp_asc 
  {
    index_cmp_asc(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { 
      return arr[a] < arr[b];
    }
    const T arr;
  };

  /** \brief  Comparison struct used by sort for descending order
    * \note: http://bytes.com/topic/c/answers/132045-sort-get-index
    */
  template<class T> struct index_cmp_dsc 
  {
    index_cmp_dsc(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { 
      return arr[a] > arr[b];
    }
    const T arr;
  };

  /** \brief Reorder a vector given a reordering index map. Act like matlab's Y = X[I]
    *  \param[in]  unordered  unsorted vector
    *  \param[in]  index_map  an index map such that sorted[i] = unsorted[index_map[i]]
    *  \return reordered vector
    *  \note: X and Y are allowed to be the same reference
    */
  template< class T >
  std::vector<T> reorder  ( const std::vector<T> &unordered,
                            const std::vector<size_t> &sort_order
                          )
  {
    // copy for the reorder according to index_map, because unsorted may also be sorted
    std::vector<T> ordered (sort_order.size());
    for(int i = 0; i<sort_order.size();i++)
    {
      ordered[i] = unordered[sort_order[i]];
    }
    
    return ordered;
  }
  
  /** \brief Sort a vector and return the reordering index map. Acts like matlab's [Y,I] = SORT(X)
    *  \param[in]  unsorted  unsorted vector
    *  \param[out] sorted    sorted vector, allowed to be same as unsorted
    *  \param[out] index_map an index map such that sorted[i] = unsorted[index_map[i]]
    *  \note: unsorted and sorted are allowed to be the same reference
    */
  template <class T>
  void sort ( std::vector<T> &unsorted,
              std::vector<T> &sorted,
              std::vector<size_t> &sort_order,
              const SortMode sort_mode = ASCENDING
            )
  {
    // Original unsorted index map
    sort_order.resize(unsorted.size());
    for(size_t i=0;i<unsorted.size();i++)
    {
      sort_order[i] = i;
    }
    // Sort the index map, using unsorted for comparison
    if (sort_mode == ASCENDING)
      sort(
        sort_order.begin(), 
        sort_order.end(), 
        index_cmp_asc<std::vector<T>& >(unsorted));
      
    else if (sort_mode == DESCENDING)
      sort(
        sort_order.begin(), 
        sort_order.end(), 
        index_cmp_dsc<std::vector<T>& >(unsorted));    
      
    else
    {
      std::cout << "[utl::sort] Unknown sort order. Assuming ascending" << std::endl;
      sort(
        sort_order.begin(), 
        sort_order.end(), 
        index_cmp_asc<std::vector<T>& >(unsorted));    
    }

    sorted = reorder(unsorted, sort_order);
  }
  
  /** \brief Convert subscripts of an element in a vector into corresponding
    * linear index.
    *  \param[in]  v       input vector
    *  \param[in]  sub1    outer subscript of the element
    *  \param[in]  sub2    inner subscript of the element
    *  \param[out] lin_id  linear index of the element
    *  \return FALSE if subscripts are out of bounds of the vector of vectors
    */    
  template <typename TypeT>
  bool vector2dSubToLinearId  ( const std::vector<TypeT> v, const int sub1, const int sub2, int &lin_id  )
  {
    if (sub1 < 0 || sub1 > v.size()-1)
    {
      std::cout << "[utl::vector2dSub2LinearId] first subscript is out of bounds." << std::endl;
      std::cout << "[utl::vector2dSub2LinearId] vector size: " << v.size() << ", subscript 1: " << sub1 << std::endl;
      return false;
    }
    
    if (sub2 < 0 || sub2 > v[sub1].size()-1)
    {
      std::cout << "[utl::vector2dSub2LinearId] second subscript is out of bounds." << std::endl;
      std::cout << "[utl::vector2dSub2LinearId] vector[s1] size: " << v[sub1].size() << ", subscript 2: " << sub2 << std::endl;
      return false;
    }
    
    lin_id = 0;
    for (size_t vId = 0; vId < sub1; vId++)
      lin_id += v[vId].size();
    
    lin_id+= sub2;
    
    return true;
  }

  /** \brief Convert a linear index of an element in a vector of vectors into
    * corresponding subscripts.
    *  \param[in]  v       input vector
    *  \param[in]  lin_id  linear index of the element
    *  \param[out] sub1    outer subscript of the element
    *  \param[out] sub2    inner subscript of the element     
    *  \return FALSE if linear index is out of bounds of the vector of vectors
    */    
  template <typename TypeT>
  bool vector2dLinearId2Sub  ( const std::vector<TypeT> v, const int lin_id, int &sub1, int &sub2  )
  {
    int curOffset = 0;
    for (sub1 = 0; sub1 < v.size(); sub1++)
    {
      curOffset += v[sub1].size();
      if (lin_id < curOffset)
      {
        sub2 = lin_id - (curOffset - v[sub1].size());
        return true;
      }
    }
    
    std::cout << "[utl::vector2dLinearId2Sub] linear index is out of bounds." << std::endl;
    std::cout << "[utl::vector2dSub2LinearId] number of elements in vector: " << curOffset << ", linear index: " << lin_id << std::endl;

    return false;
  }    
  
  //--------------------------------------------------------------------------
  // Vector filtering
  //--------------------------------------------------------------------------

  /** \brief Remove all instances of an element from vector
    *  \param[in] v  vector
    *  \param[in] el element to be removed
    */        
  template <typename TObject>
  inline
  void removeElement (std::vector<TObject> &v, const TObject &el)
  {
    v.erase(std::remove(v.begin(), v.end(), el), v.end());
  }
  
  /** \brief Make vector unique value. Values are sorted.
    *  \param[in,out] v  vector
    */        
  template <typename TObject>
  inline
  void uniqueVector (std::vector<TObject> &v)
  {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }
  
  /** \brief Remove elements from vector given indices of the elements to be removed
    *  \param[in] v  vector
    *  \param[in] indices int vector where each element represents the index of an element in the original vector that needs to be kept
    *  \return filtered vector
    */        
  template <typename TObject>
  inline
  std::vector<TObject> vectorFilter (const std::vector<TObject> &v, const std::vector<int> &indices)
  {
    // Check that indices are within the range of the vector size
    if (vectorMax(indices) > v.size()-1 || vectorMin(indices) < 0)
    {
      std::cout << "[utl::vectorFilter] indices are outside vector size range (vector size:" 
                << v.size() << ", indices range: (" << vectorMax(indices) << ", " << vectorMin(indices) << "))\n";
      return std::vector<TObject>(0);
    }
    
    // Filter
    std::vector<TObject> v_filtered;
    for (std::vector<int>::const_iterator it = indices.begin(); it != indices.end(); it++)
      v_filtered.push_back(v[*it]);
    
    return v_filtered;
  }
  
  /** \brief Remove elements from vector given a bool vector
    *  \param[in] v  vector
    *  \param[in] mask bool vector where 'false' indicates that corresponding element must be removed
    *  \return filtered vector
    */        
  template <typename TObject>
  inline
  std::vector<TObject> vectorFilter (const std::vector<TObject> &v, const std::vector<bool> &mask)
  {
    // Check that mask 
    if (v.size() != mask.size())
    {
      std::cout << "[utl::vectorFilter] vector and mask must be the same size (" << v.size() << ", " << mask.size() << ")\n";
      return std::vector<TObject>(0);
    }
    
    // Filter
    std::vector<TObject> v_filtered;
    for (size_t elId = 0; elId < mask.size(); elId++)
      if (mask[elId])
        v_filtered.push_back(v[elId]);
    
    return v_filtered;
  }
  
  //--------------------------------------------------------------------------
  // Vector set operations
  //--------------------------------------------------------------------------
  
  /** \brief Append a vector to another vector
    *  \param[in,out]  v1  vector that will be appended
    *  \param[in]      v2  element to be appended
    */        
  template <typename TObject>
  inline
  void vectorAppend (std::vector<TObject> &v1, const std::vector<TObject> &v2)
  {
    v1.insert(v1.end(), v2.begin(), v2.end());
  }
  
  /** \brief Find an intersection between two vectors. The return vector contains unique values that are present in both vectors.
    *  \param[in] v1 first vector
    *  \param[in] v2 second vector
    *  \return vector intersection
    */        
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorIntersection(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    std::vector<int> v_intersection;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());
    std::set_intersection(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_intersection));
    
    return v_intersection;
  }

  /** \brief Find a union of two vectors. The return vector contains unique values that are present in both vectors.
    *  \param[in] v1 first vector
    *  \param[in] v2 second vector
    *  \return vector union
    */        
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorUnion(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    
    std::vector<int> v_union;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());  
    v1_sorted.erase(std::unique(v1_sorted.begin(), v1_sorted.end()), v1_sorted.end());
    v2_sorted.erase(std::unique(v2_sorted.begin(), v2_sorted.end()), v2_sorted.end());
    std::set_union(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_union));
    
    return v_union;
  }

  /** \brief Find a difference between two vectors. The return vector contains unique values that are present in first vector but not in the second
    *  \param[in] v1 first vector
    *  \param[in] v2 second vector
    *  \return vector difference
    */
  template <typename Scalar>
  inline
  std::vector<Scalar> vectorDifference(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2)
  {
    std::vector<int> v_difference;
    std::vector<int> v1_sorted(v1);
    std::vector<int> v2_sorted(v2);
    std::sort(v1_sorted.begin(), v1_sorted.end());
    std::sort(v2_sorted.begin(), v2_sorted.end());  
    v1_sorted.erase(std::unique(v1_sorted.begin(), v1_sorted.end()), v1_sorted.end());
    v2_sorted.erase(std::unique(v2_sorted.begin(), v2_sorted.end()), v2_sorted.end());  
    std::set_difference(v1_sorted.begin(), v1_sorted.end(), v2_sorted.begin(), v2_sorted.end(), std::back_inserter(v_difference));
      
    return v_difference;
  }
  
  //--------------------------------------------------------------------------
  // File I/O
  //--------------------------------------------------------------------------
  
  /** \brief Write a vector of variables to a file.
    *  \param[in] v vector
    *  \param[in] filename filename
    *  \return true if writing was successfull
    */
  template <typename Scalar>
  inline
  bool writeVectorToFileASCII(const std::vector<Scalar> v, const std::string &filename)
  {
    std::ofstream file(filename);
    if (!file.is_open())
    {
      std::cout << "[utl::writeVectorToFileASCII] Could not open file for writing ('" << filename << "')." << std::endl;
      return false;
    }
    else
    {
      for (size_t elId = 0; elId < v.size(); elId++)
        file << v[elId] << std::endl;
    }
    file.close();
  }

  /** \brief Read a vector of variables from a file.
    * \param[in] v vector
    * \param[in] filename filename
    * \return true if reading was successfull
    */
  template <typename Scalar>
  inline
  bool readVectorFromFileASCII(std::vector<Scalar> &v, const std::string &filename)
  {
    v.resize(0);
    
    std::ifstream file(filename);
    
    if (!file.is_open())
    {
      std::cout << "[utl::readVectorToFileASCII] Could not open file for reading ('" << filename << "')." << std::endl;
      return false;
    }
    else
    {
      std::string line;
      Scalar var;
      while (std::getline(file, line))
      {
        if (!line.empty())
        {
          std::stringstream s(line);
          s >> var;
          v.push_back(var);
        }
      }
    }
    file.close();
    
    return true;
  }    
  
  //--------------------------------------------------------------------------
  // Element occurence count
  //--------------------------------------------------------------------------
              
  /** \brief Compute number of times a value occurs in a vector
    *  \param[in]  v a vector of values
    *  \param[in]  target_value value to be searched for
    *  \return number of ocurrences of a value in the vector
    */
  template <typename Scalar>
  inline
  size_t vectorCount(const std::vector<Scalar> &v, const Scalar target_value)
  { 
    return std::count(v.begin(), v.end(), target_value);
  }

  /** \brief Find indices of vector elements that are equal to a target value.
    *  \param[in]  v a vector of values
    *  \param[in]  target_value value to be searched for
    *  \param[in]  target_loc indices of elements equal to target value
    *  \return number of elements equal to the target value
    */
  template <typename Scalar>
  inline
  int vectorFind(const std::vector<Scalar> &v, const Scalar target_value, std::vector<int> &target_loc)
  { 
    target_loc.resize(0);
    auto it = v.begin();
    bool done = false;
    
    while (!done)
    {
      it = std::find(it, std::end(v), target_value);
      if (it == std::end(v))
        done = true;
      else
      {
        target_loc.push_back(it - v.begin());
        it++;
      }
    }
    return target_loc.size();
  }    
  
  /** \brief Count occurences of values in a vector
    *  \param[in]  v a vector of values
    *  \param[out] v_unique unique values in the vector
    *  \param[out] counts occurence counts for unique values
    *  \param[in]  sort_mode ordering mode used for sorting v_unique
    */
  template <typename Scalar>
  inline
  void vectorHistogram(const std::vector<Scalar> &v, std::vector<Scalar> &v_unique, std::vector<size_t> &counts, const SortMode sort_mode = UNSORTED)
  {
    // First get all the unique values of the vector
    v_unique = v;
    uniqueVector<Scalar>(v_unique);
    
    // Count the occurence of each value
    counts.resize(v_unique.size());
    for (size_t i = 0; i < v_unique.size(); i++)
      counts[i] = vectorCount(v, v_unique[i]);
    
    // Sort
    if (sort_mode == ASCENDING || sort_mode == DESCENDING)
    {
      std::vector<size_t> sort_order;
      utl::sort<size_t>(counts, counts, sort_order, sort_mode);
      v_unique = reorder<Scalar>(v_unique, sort_order);
    }
  }

  /** \brief Find the most (or least) freqent element in a vector
    *  \param[in]  v a vector of values
    *  \param[in]  sort_mode DESCENDING return most frequent while ASCENDING returns least frequent element
    *  \return most (or least) freqent element in a vector
    */
  template <typename Scalar>
  inline
  void vectorMode(const std::vector<Scalar> &v, Scalar &vector_mode, size_t &mode_count, const SortMode sort_mode = DESCENDING)
  {
    std::vector<Scalar> v_unique;
    std::vector<size_t> counts;
    vectorHistogram<Scalar>(v, v_unique, counts, sort_mode);
    vector_mode = v_unique[0];
    mode_count  = counts[0];
  }   
  
  //--------------------------------------------------------------------------
  // Vector minimum and maximum values
  //--------------------------------------------------------------------------
      
  /** \brief Get the maximum value of a vector of scalars
    *  \param[in] v  vector
    *  \return maximum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMax (const std::vector<Scalar> &v)
  {
    if (v.size() == 0)
    {
      std::cout << "[vectorMax] input vector is empty!" << std::endl;
      assert(false);
    }
    
    return *std::max_element(v.begin(), v.end());
  }  
  
  /** \brief Get the minimum value of a vector of scalars
    *  \param[in] v  vector
    *  \return minimum value
    */        
  template <typename Scalar>
  inline
  Scalar vectorMin (const std::vector<Scalar> &v)
  {    
    if (v.size() == 0)
    {
      std::cout << "[vectorMin] input vector is empty!" << std::endl;
      assert(false);
    }
    
    return *std::min_element(v.begin(), v.end());
  }
  
  /** \brief Get the maximum value of a vector of scalars and the indices of
    * the elements equal to the maximum value
    *  \param[in] v  vector
    *  \param[out] max_val maximum value
    *  \param[out] max_val_loc indices of elements equal to the maximum value
    *  \return number of elements equal to the maximum value
    *  \note not an efficient implementation
    */        
  template <typename Scalar>
  inline
  int vectorMaxLoc (const std::vector<Scalar> &v, Scalar &max_val, std::vector<int> &max_val_loc)
  {
    if (v.size() == 0)
    {
      max_val = 0;
      max_val_loc.resize(0);
      return -1;
    }
    
    max_val = vectorMax<Scalar>(v);
    return vectorFind<Scalar>(v, max_val, max_val_loc);
  }

  /** \brief Get the minimum value of a vector of scalars and the indices of
    * the elements equal to the minimum value
    *  \param[in] v  vector
    *  \param[out] min_val minimum value
    *  \param[out] min_val_loc indices of elements equal to the minimum value
    *  \return number of elements equal to the minimum value
    *  \note not an efficient implementation
    */        
  template <typename Scalar>
  inline
  int vectorMinLoc (const std::vector<Scalar> &v, Scalar &min_val, std::vector<int> &min_val_loc)
  {
    if (v.size() == 0)
    {
      min_val = 0;
      min_val_loc.resize(0);
      return -1;
    }
    
    min_val = vectorMin<Scalar>(v);
    return vectorFind<Scalar>(v, min_val, min_val_loc);
  }
  
  /** \brief Given a value find positions of nearest smaller and nearest greater
    * value in a vector. If only 
    *  \param[in] vec    vector
    *  \param[in] value  query value
    *  \return indices of smaller and greater values. If one of them is not available -1 is returned.
    *  \note udenfined behavior if vector contains NaNs.
    */            
  template <typename Scalar>
  inline    
  std::pair<int, int> nearestValues (const std::vector<Scalar> &vec, const Scalar value)
  {
    float minDistanceSmaller = std::numeric_limits<Scalar>::max();
    float minDistanceGreater = std::numeric_limits<Scalar>::max();
    int nearestSmallerIndex = -1;
    int nearestGreaterIndex = -1;
    
    for (size_t i = 0; i < vec.size(); i++)
    {
      if (vec[i] < value)
      {
        Scalar distance = value - vec[i];
        if (distance < minDistanceSmaller)
        {
          minDistanceSmaller = distance;
          nearestSmallerIndex = i;
        }
      }
      else if (vec[i] > value)
      {
        Scalar distance = vec[i] - value;
        if (distance < minDistanceGreater)
        {
          minDistanceGreater = distance;
          nearestGreaterIndex = i;
        }          
      }
      else if (vec[i] == value)
      {
        return std::pair<int,int>(i,i);
      }
    }
    
    return std::pair<int,int>(nearestSmallerIndex, nearestGreaterIndex);
  }
}

#endif // STD_VECTOR_UTILITIES_HPP