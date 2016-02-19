/**
 * \file WocraDebug.h
 * \author Sovannara Hak, Joseph Salini
 *
 * \brief Debug trace
 *
 * Inspired by François Bleibel, Olivier Stasse
 * define WOCRA_DEBUG to activate tracing
 * define trace level WOCRA_DEBUG_MODE
 * use isirDEBUG(level) or isirTOTALDEBUG(level) like ostream
 */

#ifndef WocraDEBUG_H
# define WocraDEBUG_H

//#define WOCRA_DEBUG
#define WOCRA_DEBUG_MODE 0


# include <cstdio>
# include <iostream>
# include <fstream>
# include <sstream>

# ifndef WOCRA_DEBUG_MODE
#  define WOCRA_DEBUG_MODE 0
# endif

class WocraDebugTrace{
	public :
		WocraDebugTrace(std::ostream& os);

		std::ostream& outputbuffer;

};
extern WocraDebugTrace isirDEBUGFLOW;

# ifdef WOCRA_DEBUG
#  define isirPREDEBUG 					\
	__FILE__ << ": " << __FUNCTION__ 	\
	<< "(#" << __LINE__ << ") :"

#  define isirTOTALDEBUG(level)													\
	if ((level>WOCRA_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer << isirPREDEBUG

#  define isirDEBUG(level)													\
	if ((level>WOCRA_DEBUG_MODE) || (!isirDEBUGFLOW.outputbuffer.good()) )	\
	;																		\
	else isirDEBUGFLOW.outputbuffer
# else // WOCRA_DEBUG
#  define isirTOTALDEBUG(level) if( 1 ) ; else std::cout
#  define isirDEBUG(level) if( 1 ) ; else std::cout

#endif // WOCRA_DEBUG

#endif //ifndef WocraDEBUG_H
