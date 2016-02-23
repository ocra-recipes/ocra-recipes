/**
 * \file wOcraDebug.h
 * \author Sovannara Hak, Joseph Salini
 *
 * \brief Debug trace
 *
 * Inspired by François Bleibel, Olivier Stasse
 * define wOcra_DEBUG to activate tracing
 * define trace level wOcra_DEBUG_MODE
 * use isirDEBUG(level) or isirTOTALDEBUG(level) like ostream
 */

#ifndef wOcraDEBUG_H
# define wOcraDEBUG_H

//#define wOcra_DEBUG
#define wOcra_DEBUG_MODE 0


# include <cstdio>
# include <iostream>
# include <fstream>
# include <sstream>

# ifndef wOcra_DEBUG_MODE
#  define wOcra_DEBUG_MODE 0
# endif

class IsirDebugTrace{
	public :
		IsirDebugTrace(std::ostream& os);

		std::ostream& outputbuffer;

};
extern IsirDebugTrace isirDEBUGFLOW;

# ifdef wOcra_DEBUG
#  define isirPREDEBUG 					\
	__FILE__ << ": " << __FUNCTION__ 	\
	<< "(#" << __LINE__ << ") :"

#  define isirTOTALDEBUG(level)													\
	if ((level>wOcra_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer << isirPREDEBUG

#  define isirDEBUG(level)													\
	if ((level>wOcra_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer
# else // wOcra_DEBUG
#  define isirTOTALDEBUG(level) if( 1 ) ; else std::cout
#  define isirDEBUG(level) if( 1 ) ; else std::cout

#endif // wOcra_DEBUG

#endif //ifndef wOcraDEBUG_H
