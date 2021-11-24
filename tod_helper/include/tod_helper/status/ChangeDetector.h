// Copyright 2021 Feiler

#pragma once

#include "tod_msgs/Status.h"

namespace tod_helper::Status {
class TodStatusChangeDetector {
public:
    virtual ~TodStatusChangeDetector() = default;
    TodStatusChangeDetector(tod_msgs::Status* status, const uint8_t conStatus);
    bool statusChanged();
private:
    uint8_t _statusUnderObservation;
    uint8_t _previousState;
    tod_msgs::Status* _status;
};

inline TodStatusChangeDetector::TodStatusChangeDetector(tod_msgs::Status* status,
                                                  const uint8_t conStatus) :
    _statusUnderObservation(conStatus), _previousState(status->tod_status),
    _status(status) { }

inline bool TodStatusChangeDetector::statusChanged() {
    bool statusChanged = false;
    uint8_t currentStatus = _status->tod_status;
    if ( _previousState != currentStatus ) {
        statusChanged = true;
    } else {
        statusChanged = false;
    }
    _previousState = currentStatus;
    return statusChanged;
}

}; // namespace tod_helper::Status
