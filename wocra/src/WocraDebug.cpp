/**
 * \file WocraDebug.cpp
 * \author Joseph Salini
 *
 * \brief Implementation to get debugging information.
 */

#include "wocra/WocraDebug.h"
#include <fstream>
#include <ios>


WocraDebugTrace isirDEBUGFLOW(std::cout);

WocraDebugTrace::WocraDebugTrace(std::ostream& os)
    : outputbuffer(os){
}
