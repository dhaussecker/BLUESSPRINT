#ifndef NOTECARD_TIME_H
#define NOTECARD_TIME_H

#include <Notecard.h>

struct TimestampResult {
    unsigned long unixTime;
    bool success;
};

// Get current Unix timestamp from Blues Notecard
// Returns: TimestampResult with unixTime and success flag
// success = true if timestamp was retrieved successfully
// success = false if failed to get timestamp
TimestampResult getNotecardTimestamp(Notecard& notecard);

#endif // NOTECARD_TIME_H