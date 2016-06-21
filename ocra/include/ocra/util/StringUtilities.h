#ifndef OCRA_UTIL_STRING_UTILITIES_H
#define OCRA_UTIL_STRING_UTILITIES_H

#include <algorithm>
#include <string>

namespace ocra {
    namespace util {


inline std::string convertToLowerCase(const std::string& originalString)
{
    std::string newString = originalString;
    std::transform(newString.begin(), newString.end(), newString.begin(), ::tolower);
    return newString;
}





    } // namespace ocra
} // namespace util
#endif // OCRA_UTIL_STRING_UTILITIES_H
