
#ifndef THE_NAMING_HPP_9841204810AH9341234
#define THE_NAMING_HPP_9841204810AH9341234

#include <string>

namespace as2 { namespace mc {

class TheNaming 
{
public:
  static std::string name(size_t) 
  {
    return std::string("/as2_mc/example/pose");
  }
};

} } //namespace

#endif