/**
 * \file Performances.h
 * \author Joseph Salini
 *
 * \brief Define and implement function to compute and save performances of the GHCJT controlker.
 */

#ifndef __PERFORMANCES_H__
#define __PERFORMANCES_H__

#include <sys/time.h>

#include <list>

#include <iostream>

namespace gocra
{

/** \addtogroup core
 * \{
 */

/** \brief To get and save time information to collect some data on performances.
 *
 * It can be used in two ways. First to save a timeline:
 *
 * \code
 *
 * PerformanceRecorder     perfRec;    // create performance recorder
 *
 * ...
 *
 * perfRec.initializeTime();             // initialize somewhere
 *
 * for ( ... )
 * {
 *     perfRec.saveRelativeTime();       // save time relatively the the call of initializeTime()
 *
 *     ...
 * }
 *
 * \endcode
 *
 * Or to save loop performances:
 *
 * \code
 *
 * PerformanceRecorder     perfRec;    // create performance recorder
 *
 * ...
 *
 * for ( ... )
 * {
 *     perfRec.initializeTime();       // reset time at the beginning of the loop
 *
 *     ...
 *
 *     perfRec.saveRelativeTime();     // save time at the end of the loop
 * }
 *
 * \endcode
 *
 */
class PerformanceRecorder
{
public:

    /** Create a performance recorder. It also initializes the internal initial time.
     *
     */
    PerformanceRecorder()           { initializeTime(); };

    /** Destructor
     *
     */
    virtual ~PerformanceRecorder()  {};

    /** Get the current time.
     *
     * It calls method "gettimeofday" and returns a value representing the current time of the day in second.
     */
    double getCurrentTime() {
        gettimeofday(&tim, NULL);
        return tim.tv_sec+(tim.tv_usec/1000000.0);
    };

    /** Initialize internal Zero time.
     *
     * It calls method #getCurrentTime() to get a time reference.
     */
    void initializeTime() {
        _initTime = getCurrentTime();
    };

    /** Save the relative elapsed time since the time reference, i.e. the last call of #initializeTime().
     *
     * The value is saved in a list available with #getSavedTime() const
     */
    void saveRelativeTime() {
        double t = getCurrentTime();
        _record.push_back( t - _initTime );
    };

    /** get the relative elapsed time since the time reference, i.e. the last call of #initializeTime().
     *
     */
    double getRelativeTime() {
        return getCurrentTime() - _initTime;
    };

    void saveTime(double t)
    {
        _record.push_back( t );
    }

    /** Get the list where are saved the relative times.
     *
     */
    const std::list< double >& getSavedTime() const {
        return _record;
    }

    /** Save performances information in a output stream.
     *
     * \param headerName The name of the performance saved in the
     * \param outstream the output stream where to write the performances information
     * \param addCommaAtEnd If true, add a comma at the end of the stream. If false, it means that this is the end of the json file, nothing will be added after that, no comma is added.
     */
    void writeInStream(const std::string& headerName, std::ostream& outstream, bool addCommaAtEnd) {
        outstream << "\"" <<headerName << "\": [";

        std::list< double >::const_iterator it_1 = --_record.end();
        for (std::list< double >::const_iterator it = _record.begin(); it != _record.end(); ++it)
        {
            outstream << *it;
            if ( it != it_1)
                outstream << ", ";
        }
        if (addCommaAtEnd == true)
            outstream << "],\n";
        else
            outstream << "]\n";
    }

protected:
    timeval tim;

    double _initTime;

    std::list< double > _record;
};


/** \} */ // end group core


}

#endif



