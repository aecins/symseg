// Copyright 2017 Aleksandrs Ecins
// Licensed under GPLv2+
// Refer to the LICENSE.txt file included.

#ifndef FILESYSTEM_UTILITIES_HPP
#define FILESYSTEM_UTILITIES_HPP

// STD includes
#include <iostream>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/regex.hpp>

// Other
 #include <filesystem/alphanum.hpp>

namespace utl
{
  /** \brief Replace all occurences of a substring with a new substring
    *  \param[in,out] string_in string which is modified
    *  \param[in] substring_orig substring that needs to be replaced
    *  \param[in] substring_new new substring
    */
  inline
  void replaceSubstring(std::string &string_in, const std::string &substring_orig, const std::string &substring_new)
  {
    std::string::size_type n = 0;
    while ( ( n = string_in.find( substring_orig, n ) ) != std::string::npos )
    {
        string_in.replace( n, substring_orig.size(), substring_new );
        n += substring_new.size();
    }  
  }  
  
  /** \brief An analogue of the MATLAB dir function
    * \param[in] dir_path input directory
    * \param[out] content_list a vector of strings representing names of files/folders in the directory
    */
  inline
  void dir(const std::string &dir_path, std::vector<std::string> &content_list)
  {
    std::string dirPath (dir_path);
    
    // First normalize the beginning of the path
    // If directory path does not start with '/', './' or '../' append './'
    if  (!(((dirPath.length() >= 1) && (dirPath.substr(0,1) == "/")) || 
        ((dirPath.length() >= 2) && (dirPath.substr(0,2) == "./")) ||
        ((dirPath.length() >= 3) && (dirPath.substr(0,3) == "../"))))
    {
      dirPath.insert(0, "./");
    }
      
    // Find the rightmost '/' delimeter and split along it
    std::string left, right;
    size_t found = dirPath.find_last_of("/");
    if (found == std::string::npos)
    {
      std::cout << "[dir] something wrong\n";
      exit(EXIT_FAILURE);
    }
    else
    {
      left = dirPath.substr(0, found+1);
      right = dirPath.substr(found+1);
    }
    
    // Now analyze the right part
    std::string fileFormatStr;
    
    // If right part is empty list everything in left
    if (right.empty())
    {
      fileFormatStr = "*";
      dirPath = left;
    }
    
    // If right side containts a wildcard or an extension list everything in left that matches
    else if ((right.find("*") != std::string::npos) || (boost::filesystem::path(right).has_extension()))
    {
      fileFormatStr = right;
      dirPath = left;
    }
    
    // If right is a directory append it to left and list everything
    else if (!boost::filesystem::path(right).has_extension())
    {
      fileFormatStr = "*";
      dirPath = left + right;
    }

    // Generate regex expression
    std::string regexExpression (fileFormatStr);
    utl::replaceSubstring(regexExpression, ".", "[.]");      // Replace all occurences of '.' with '[.]'
    utl::replaceSubstring(regexExpression, "*", ".*");       // Replace all occurences of '*' with '.*'
    boost::regex filenameFormatRgx(regexExpression);  
    
    // List
    content_list.resize(0);
    for (boost::filesystem::directory_iterator file_it(dirPath), end; file_it != end; ++file_it)
    {
      std::string curFileName = file_it->path().leaf().string();                          // Get current file name
      
      if (boost::regex_match(curFileName, filenameFormatRgx))                               // Check if it matches the filename format   
        content_list.push_back(curFileName);
    }
    
    // Sort the list
    std::sort(content_list.begin(), content_list.end(), doj::alphanum_less<std::string>());    
  }

  /** \brief Add trailing slash to pathname if it is missing
    * \param[in] pathname file path
    * \return path with trailing slash
    */
  inline
  std::string addTrailingSlash (const std::string &pathname)
  {
    if (boost::filesystem::path(pathname).has_extension())
      return pathname;
    if (!pathname.empty() && pathname.back() != '/')
      return pathname + '/';
    else
      return pathname;
  }
  
  /** \brief Add trailing slash to pathname if it is missing
    * \param[in] pathname file path
    * \return path with trailing slash
    */
  inline
  std::string removeTrailingSlash (const std::string &pathname)
  {
    if (!pathname.empty() && pathname.back() == '/')
      return pathname.substr(0, pathname.size()-1);
    else
      return pathname;
  }
  
  /** \brief Find filename extension of a path
    * \param[in] path path
    * \return extension
    */
  inline
  std::string getExtension (const std::string &pathname)
  {
    return boost::filesystem::extension(boost::filesystem::path(pathname));
  }

  
  /** \brief Find the parent directory of a path
    * \param[in] path path
    * \return parent directory
    */
  inline
  std::string getParentDir (const std::string &path)
  {
    std::string parentDir = boost::filesystem::path(removeTrailingSlash(path)).parent_path().string();
    if (parentDir == "")
      parentDir = "./";
    
    return parentDir;
  }
  
  /** \brief If input is a filename - return filename without preceeding path. 
    *  If input is path - return last directory in the path
    * \param[in] path path
    * \return filename
    */
  inline
  std::string getBasename (const std::string &path)
  {
    if (boost::filesystem::path(path).has_extension())
      return boost::filesystem::path(path).filename().string();
    else if (!path.empty() && path.back() != '/')
      return boost::filesystem::path(path).stem().string();
    else
      return boost::filesystem::path(path.substr(0, path.length()-1)).stem().string();
  }

  /** \brief Get filename without extension from the path
    * \param[in] path path
    * \return filename without extension
    */
  inline
  std::string getBasenameNoExtension (const std::string &path)
  {
    return boost::filesystem::path(path).stem().string();
  }
  
  /** \brief Check if a path exists or not
    * \return true if path exists
    */
  inline
  bool exists (const std::string &path)
  {
    return boost::filesystem::exists(path);
  }

  /** \brief Check if a path is an existing file
    * \return true if path is a filename
    */
  inline
  bool isFile (const std::string &path)
  {
    return (boost::filesystem::is_regular_file(boost::filesystem::path(path)));    
  }
  
  /** \brief Check if a path is a directory that exists
    * \return true if path is a directory
    */
  inline
  bool isDirectory (const std::string &path)
  {
    return (boost::filesystem::is_directory(boost::filesystem::path(path)));
  }  
  
  /** \brief Delete a directory and all of its contents
    * \param[in] path path
    * \return false if directory does not exist or could not delete directory
    */
  inline
  bool deleteDir (const std::string &path)
  {
    if (!boost::filesystem::remove_all(path))
    {
      std::cout << "[utl::deleteDir] could not delete directory '" << path << "'\n";
      return false;
    }
    
    return true;
  }
  
  /** \brief Create a directory and all of its contents
    * \param[in] path path
    * \return false if directory already exists or could not create directory
    */
  inline
  bool createDir (const std::string &path)
  {    
    if (!boost::filesystem::create_directory(boost::filesystem::path(path)))
    {
      std::cout << "[utl::createDir] Could not create directory '" << path << "'\n";
      return false;
    }
    
    return true;
  }
  
  /** \brief Join to paths to generate new path. Inspired by MATLAB's fullfile
    * \param[in] path1 first path
    * \param[in] path2 second path
    * \return joined path
    */
  inline
  std::string fullfile (const std::string &path1, const std::string &path2)
  {
    boost::filesystem::path joinedPath = boost::filesystem::path(path1) / boost::filesystem::path(path2);
    return joinedPath.string();
  }
}

#endif    // FILESYSTEM_UTILITIES_HPP