/**
 * \file wOcraDebug.cpp
 * \author Joseph Salini
 *
 * \brief Implementation to get debugging information.
 */

#include "wocra/wOcraDebug.h"
#include <fstream>
#include <ios>


IsirDebugTrace isirDEBUGFLOW(std::cout);

IsirDebugTrace::IsirDebugTrace(std::ostream& os)
    : outputbuffer(os){
}
