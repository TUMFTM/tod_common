// Copyright 2020 Feiler

#pragma once

#include "ModeChecker.h"
#include "tod_msgs/Status.h"

namespace tod_helper::Status {

class ControlModeChecker : public ModeChecker {
public:
    virtual ~ControlModeChecker() = default;
    ControlModeChecker(int mode, tod_msgs::Status* status);
    bool modeIsChosen() const override;
private:
    int _mode;
    tod_msgs::Status* _status;
};

inline ControlModeChecker::ControlModeChecker(int mode, tod_msgs::Status* status) :
    _mode(mode), _status(status) {
}

inline bool ControlModeChecker::modeIsChosen() const {
    if ( _status == nullptr ) {
        throw std::runtime_error("exception status_msg == nullptr");
    }
    return _status->operator_control_mode == _mode;
}

} // namespace tod_helper::Status
