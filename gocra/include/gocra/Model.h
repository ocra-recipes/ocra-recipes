/**
 * \file Model.h
 * \author Joseph Salini
 *
 * \brief Define method to load a ocra::Model directly from a shared library .
 */

#ifndef __GOCRAMODEL_H__
#define __GOCRAMODEL_H__

#include <iostream>
#include <stdlib.h> // to get relative path of file

#include <dlfcn.h> // To load shared libraries

#include "ocra/control/Model.h"


namespace gocra
{

ocra::Model* getModelFromSharedLibrary(const std::string& libPath, const std::string& createFunctionName, const std::string& robotName)
{
    char  actualpath[4096];
    realpath(libPath.c_str(), actualpath);

    void* SL_Handle;
    SL_Handle = dlopen(actualpath, RTLD_LAZY);
    if(SL_Handle == NULL){
       throw std::runtime_error(std::string("[gocra::getModelFromSharedLibrary]: Cannot load shared library: '")+std::string(actualpath)+"'" );
    }
    ocra::Model* (*create_SL_Model)(const std::string&);
    *(void **)(&create_SL_Model) = dlsym(SL_Handle, createFunctionName.c_str());

    return (*create_SL_Model)(robotName);

}


}

#endif
