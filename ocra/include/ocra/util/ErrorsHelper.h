#include <iostream>

inline std::string fullMethodName(const std::string& prettyFunction)
{
    size_t colons = prettyFunction.find("::");
    size_t begin = prettyFunction.substr(0,colons).rfind(" ") + 1;
    size_t end = prettyFunction.rfind("(") - begin;

    return "[" + prettyFunction.substr(begin,end) + "()" "]";
}

#define __FULL_METHOD_NAME__ fullMethodName(__PRETTY_FUNCTION__)

// Colors code is as follows:
// \33 at the beginning and the end correspond to the octal for ESC which must always be present.
//  1 stands for BOLD
//  30 + n is the code for the text color.
//  40 + n is the code for the background color.
//  90 + n for high intensity text color.
//  n: 0 - black
//     1 - red
//     2 - green
//     3 - yellow
//     4 - blue
//     5 - magenta
//     6 - cyan
//     7 - white

#define OCRA_ERROR(msg) \
std::cout << "\033[1;93;41m[ERROR]" << __FULL_METHOD_NAME__ << "\033[39;49m" << msg << std::endl;

#define OCRA_WARNING(msg) \
std::cout << "\033[1;97;42m[WARNING]" << __FULL_METHOD_NAME__ << "\033[39;49m" << msg << std::endl;
