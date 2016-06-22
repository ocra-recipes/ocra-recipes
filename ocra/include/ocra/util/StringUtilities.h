#ifndef OCRA_UTIL_STRING_UTILITIES_H
#define OCRA_UTIL_STRING_UTILITIES_H

#include <algorithm>
#include <string>
#include <Eigen/Dense>

namespace ocra {
    namespace util {


inline std::string convertToLowerCase(const std::string& originalString)
{
    std::string newString = originalString;
    std::transform(newString.begin(), newString.end(), newString.begin(), ::tolower);
    return newString;
}

inline std::string convertToUpperCase(const std::string& originalString)
{
    std::string newString = originalString;
    std::transform(newString.begin(), newString.end(), newString.begin(), ::toupper);
    return newString;
}

inline Eigen::VectorXd stringToVectorXd(const char * valueString)
{
    std::stringstream valueStream;
    std::vector<double> doubleVector;

    valueStream << valueString;

    do
    {
        // read as many numbers as possible.
        for (double number; valueStream >> number;) {
            doubleVector.push_back(number);
        }
        // consume and discard token from stream.
        if (valueStream.fail())
        {
            valueStream.clear();
            std::string token;
            valueStream >> token;
        }
    }
    while (!valueStream.eof());

    int nRows = doubleVector.size();
    Eigen::VectorXd eigenVector(nRows);
    for (int i=0; i<nRows; i++)
    {
        eigenVector[i] = doubleVector[i];
    }
    return eigenVector;
}


inline Eigen::VectorXi stringToVectorXi(const char * valueString)
{
    std::stringstream valueStream;
    std::vector<int> doubleVector;

    valueStream << valueString;

    do
    {
        // read as many numbers as possible.
        for (int number; valueStream >> number;) {
            doubleVector.push_back(number);
        }
        // consume and discard token from stream.
        if (valueStream.fail())
        {
            valueStream.clear();
            std::string token;
            valueStream >> token;
        }
    }
    while (!valueStream.eof());

    int nRows = doubleVector.size();
    Eigen::VectorXi eigenVector(nRows);
    for (int i=0; i<nRows; i++)
    {
        eigenVector[i] = doubleVector[i];
    }
    return eigenVector;
}




    } // namespace ocra
} // namespace util
#endif // OCRA_UTIL_STRING_UTILITIES_H
