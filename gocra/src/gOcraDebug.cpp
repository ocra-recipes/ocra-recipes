/**
 * \file gOcraDebug.cpp
 * \author Joseph Salini
 *
 * \brief Implementation to get debugging information.
 */

#include "gocra/gOcraDebug.h"
#include <fstream>
#include <ios>


IsirDebugTrace isirDEBUGFLOW(std::cout);

IsirDebugTrace::IsirDebugTrace(std::ostream& os)
    : outputbuffer(os){
}
