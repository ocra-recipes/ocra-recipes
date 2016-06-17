#ifndef OCRA_UTILS_MACROS_H
#define OCRA_UTILS_MACROS_H

#include <memory>
/*! Macro function which basically just defines pointer typdefs for classes. Note that normally macros are evil monsters but I think this is a perfect usage case to cut down on repeated code.
 *  \param Class Just pop this bad boy below the class definition and it will do the rest.
 */
 #define DEFINE_CLASS_POINTER_TYPEDEFS(Class) \
     public : using Ptr = std::shared_ptr<Class>;

#endif // OCRA_UTILS_MACROS_H
