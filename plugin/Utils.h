#pragma once

#include <algorithm>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

/** \brief Checks that a plugin config attribute exists. */
inline bool checkAttr(const std::string & input)
{
  char * end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

/** \brief Converts a string into a numeric vector. */
template<typename T>
inline void readVector(std::vector<T> & output, const std::string & input)
{
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while(getline(ss, item, delim))
  {
    if(!checkAttr(item))
    {
      continue;
    }
    output.push_back(static_cast<T>(strtod(item.c_str(), nullptr)));
  }
}
