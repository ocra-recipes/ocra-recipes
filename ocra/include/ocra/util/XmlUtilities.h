#ifndef OCRA_UTIL_XML_UTILITIES
#define OCRA_UTIL_XML_UTILITIES

#include <tinyxml.h>
#include <vector>

namespace ocra {
    namespace util {




inline std::string getDisplacementArgs(TiXmlElement* xmlElem)
{
    std::string dispString;

    if(xmlElem != NULL)
    {
        std::vector<std::string> dispAttributes;

        dispAttributes.push_back("x");
        dispAttributes.push_back("y");
        dispAttributes.push_back("z");
        dispAttributes.push_back("qw");
        dispAttributes.push_back("qx");
        dispAttributes.push_back("qy");
        dispAttributes.push_back("qz");

        std::vector<std::string>::iterator stIt;

        for (stIt=dispAttributes.begin(); stIt != dispAttributes.end(); stIt++)
        {
            bool haveAlreadyFoundValue = false;
            for (TiXmlAttribute* xmlAttrib=xmlElem->FirstAttribute(); xmlAttrib != NULL; xmlAttrib = xmlAttrib->Next())
            {
                // std::cout << "stIT " << *stIt  << " xmlAttrib->Value() " << xmlAttrib->Value()<< std::endl;
                if ( (*stIt == xmlAttrib->Name()) && (!haveAlreadyFoundValue) )
                {
                    dispString += xmlAttrib->Value();
                    dispString += " ";
                    haveAlreadyFoundValue = true;
                }
            }
            if (!haveAlreadyFoundValue) {
                std::string fallbackString = "0.0 ";
                if (*stIt == "qw") {
                    fallbackString = "1.0 ";
                }
                dispString += fallbackString;
            }
        }

    }

    return dispString;
}


    } // namespace util
} // namespace ocra
#endif // OCRA_UTIL_XML_UTILITIES
