/**
 * \file gOcraDebug.h
 * \author Sovannara Hak, Joseph Salini
 *
 * \brief Debug trace
 *
 * Inspired by François Bleibel, Olivier Stasse
 * define gOcra_DEBUG to activate tracing
 * define trace level gOcra_DEBUG_MODE
 * use isirDEBUG(level) or isirTOTALDEBUG(level) like ostream
 */

#ifndef gOcraDEBUG_H
# define gOcraDEBUG_H

//#define gOcra_DEBUG
#define gOcra_DEBUG_MODE 0


# include <cstdio>
# include <iostream>
# include <fstream>
# include <sstream>

# ifndef gOcra_DEBUG_MODE
#  define gOcra_DEBUG_MODE 0
# endif

class IsirDebugTrace{
	public :
		IsirDebugTrace(std::ostream& os);

		std::ostream& outputbuffer;

};
extern IsirDebugTrace isirDEBUGFLOW;

# ifdef gOcra_DEBUG
#  define isirPREDEBUG 					\
	__FILE__ << ": " << __FUNCTION__ 	\
	<< "(#" << __LINE__ << ") :"

#  define isirTOTALDEBUG(level)													\
    if ((level>gOcra_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer << isirPREDEBUG

#  define isirDEBUG(level)													\
    if ((level>gOcra_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer
# else // gOcra_DEBUG
#  define isirTOTALDEBUG(level) if( 1 ) ; else std::cout
#  define isirDEBUG(level) if( 1 ) ; else std::cout

#endif // gOcra_DEBUG

#endif //ifndef gOcraDEBUG_H
