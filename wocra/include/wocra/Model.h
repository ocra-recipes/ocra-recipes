/**
 * \file Model.h
 * \author Joseph Salini
 *
 * \brief Define method to load a ocra::Model directly from a shared library .
 */

#ifndef __WOCRAMODEL_H__
#define __WOCRAMODEL_H__

#include <iostream>
#include <stdlib.h> // to get relative path of file

#include <dlfcn.h> // To load shared libraries

#include "ocra/control/Model.h"




namespace wocra
{


/** \addtogroup core
 * \{
 */
ocra::Model* getModelFromSharedLibrary(const std::string& libPath, const std::string& createFunctionName, const std::string& robotName)
{
    char  actualpath[4096];
    realpath(libPath.c_str(), actualpath);

    void* SL_Handle;
    SL_Handle = dlopen(actualpath, RTLD_LAZY);
    std::cout << "loading" << std::endl;
    if(SL_Handle == NULL){
       throw std::runtime_error(std::string("[wocra::getModelFromSharedLibrary]: Cannot load shared library: '")+std::string(actualpath)+"'" );
    }
    ocra::Model* (*create_SL_Model)(const std::string&);
    std::cout << "loading" << std::endl;
    *(void **)(&create_SL_Model) = dlsym(SL_Handle, createFunctionName.c_str());
    std::cout << "loading" << std::endl;

    return (*create_SL_Model)(robotName);

}

/** \} */ // end group core



} //end of wocra

#endif
