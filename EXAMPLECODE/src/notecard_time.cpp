#include "notecard_time.h"

TimestampResult getNotecardTimestamp(Notecard& notecard) {
    TimestampResult result = {0, false};

    // Request current time from Notecard
    J *req = notecard.newRequest("card.time");
    if (req == NULL) {
        return result;
    }

    // Get response
    J *rsp = notecard.requestAndResponse(req);
    if (rsp == NULL) {
        return result;
    }

    // Check for time field in response
    if (JHasObjectItem(rsp, "time")) {
        result.unixTime = JGetNumber(rsp, "time");
        result.success = true;
    }

    // Clean up response
    notecard.deleteResponse(rsp);

    return result;
}